#![no_main]
#![no_std]

use panic_semihosting as _;
// use panic_halt as _;
// use panic_abort as _;

// use cortex_m::asm;
use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;

use core::fmt::Write;

use embedded_graphics::{
    // image::{Image, ImageRaw},
    mono_font::{ascii::FONT_5X8, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::{HistoryBuffer, String};

use pid_control::{self, Controller, PIDController};

use l298n;
use mpu6050::*;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use bitbang_hal::i2c;
use hal::{
    delay::Delay,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    qei::QeiOptions,
    time::U32Ext,
    timer::{Tim1NoRemap, Tim2NoRemap, Tim4NoRemap, Timer},
};
use stm32f1xx_hal as hal;

const DT_MS: u32 = 1;

#[entry]
fn main() -> ! {
    let (dp, cp) = (
        pac::Peripherals::take().unwrap(),
        cortex_m::Peripherals::take().unwrap(),
    );

    let (mut flash, mut rcc) = (dp.FLASH.constrain(), dp.RCC.constrain());
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    let (mut gpioa, mut gpiob) = (dp.GPIOA.split(&mut rcc.apb2), dp.GPIOB.split(&mut rcc.apb2));

    let mut motor = {
        let tim1_pins = (
            gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh),
            gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh),
        );
        let (ain1, ain2, bin1, bin2) = (
            gpiob.pb13.into_push_pull_output(&mut gpiob.crh).downgrade(),
            gpiob.pb12.into_push_pull_output(&mut gpiob.crh).downgrade(),
            gpiob.pb14.into_push_pull_output(&mut gpiob.crh).downgrade(),
            gpiob.pb15.into_push_pull_output(&mut gpiob.crh).downgrade(),
        );
        let (pwm_b, _, _, pwm_a) = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2)
            .pwm::<Tim1NoRemap, _, _, _>(tim1_pins, &mut afio.mapr, 1.khz())
            .split();

        let mut motor = l298n::L298N::new(ain1, ain2, pwm_a, bin1, bin2, pwm_b);
        motor.a.forward();
        motor.b.forward();
        motor
    };

    let (qei_a, qei_b) = (
        Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).qei::<Tim2NoRemap, _>(
            (gpioa.pa0, gpioa.pa1),
            &mut afio.mapr,
            QeiOptions::default(),
        ),
        Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1).qei::<Tim4NoRemap, _>(
            (gpiob.pb6, gpiob.pb7),
            &mut afio.mapr,
            QeiOptions::default(),
        ),
    );
    let mut count_qei_a = 0;
    let mut count_qei_b = 0;

    let mut display = {
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let i2c = BlockingI2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );
        let interface = I2CDisplayInterface::new(i2c);

        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display
    };

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X8)
        .text_color(BinaryColor::On)
        .build();

    let mut mpu = {
        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let sda = pb3.into_open_drain_output(&mut gpiob.crl);
        let scl = pb4.into_open_drain_output(&mut gpiob.crl);
        let tmr = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(800.khz());
        let i2c = i2c::I2cBB::new(scl, sda, tmr);
        let mut mpu = Mpu6050::new(i2c);
        mpu.init(&mut delay).ok();
        mpu
    };

    let (mut pid_pitch, mut pid_gyro, mut pid_spd, mut _pid_yaw) = (
        /* Pitch - P */
        PIDController::new((13.5 * 1.0) as f64, 0 as f64, 0 as f64),
        /* Gyro - D */
        PIDController::new(0.05 as f64, 0 as f64, 0 as f64),
        /* SPD - PD */
        PIDController::new(-0.0003 as f64, -0.00001 as f64, 0 as f64),
        /* Yaw */
        PIDController::new(0 as f64, 0 as f64, 0 as f64),
    );
    pid_pitch.set_limits(-8000 as f64, 8_000 as f64);
    pid_pitch.set_target(0 as f64);

    pid_gyro.set_limits(-4000 as f64, 4000 as f64);
    pid_gyro.set_target(0 as f64);

    pid_spd.set_limits(-100 as f64, 100 as f64);
    pid_spd.set_target(0 as f64);

    let mut buf_pitch = HistoryBuffer::<_, 2>::new();
    let mut buf_pitch_gyro = HistoryBuffer::<_, 16>::new();

    loop {
        display.clear();

        match (mpu.get_acc_angles(), mpu.get_gyro()) {
            (Ok(angles), Ok(gyros)) => match (angles.get(1), gyros.get(1)) {
                (Some(&pitch), Some(&gyro_pitch)) => {
                    /* Update MPU6050 Data */
                    buf_pitch.write((pitch * 1000.0) as i32 - 13);
                    buf_pitch_gyro.write(gyro_pitch as i32 - 520);

                    /* Update QEI and spd */
                    let count_now = qei_a.count();
                    let spd_a = count_now.wrapping_sub(count_qei_a) as i16;
                    count_qei_a = count_now;
                    let count_now = qei_b.count();
                    let spd_b = count_now.wrapping_sub(count_qei_b) as i16;
                    count_qei_b = count_now;
                    let spd = spd_a - spd_b;

                    /* Use spd_output to set the pid_pitch's target */
                    let spd_output = pid_spd.update(spd as f64, DT_MS as f64);
                    pid_pitch.set_target(spd_output);

                    let pitch_avg = (buf_pitch.iter().sum::<i32>() / buf_pitch.len() as i32) as f64;
                    let gyro_avg =
                        (buf_pitch_gyro.iter().sum::<i32>() / buf_pitch_gyro.len() as i32) as f64;

                    let pitch_output = pid_pitch.update(pitch_avg, DT_MS as f64);
                    let gyro_output = pid_gyro.update(gyro_avg, DT_MS as f64);

                    let output_l = (pitch_output + gyro_output) as i16;
                    let output_r = (pitch_output + gyro_output) as i16;
                    motor_output(&mut motor, output_l, output_r);

                    let mut s: String<64> = String::new();
                    writeln!(s, "Pitch={:1.4}", pitch_avg).unwrap();
                    Text::with_baseline(&s, Point::zero(), text_style, Baseline::Top)
                        .draw(&mut display)
                        .unwrap();

                    let mut s: String<64> = String::new();
                    writeln!(s, "Gyro={}", gyro_avg).unwrap();
                    Text::with_baseline(&s, Point::new(0, 8), text_style, Baseline::Top)
                        .draw(&mut display)
                        .unwrap();

                    let mut s: String<64> = String::new();
                    writeln!(s, "Spd={}", spd).unwrap();
                    Text::with_baseline(&s, Point::new(0, 16), text_style, Baseline::Top)
                        .draw(&mut display)
                        .unwrap();

                    display.flush().unwrap();
                }
                _ => continue,
            },
            _ => {}
        }
    }
}

fn motor_output(
    motor: &mut l298n::L298N<
        hal::gpio::Pxx<hal::gpio::Output<hal::gpio::PushPull>>,
        hal::gpio::Pxx<hal::gpio::Output<hal::gpio::PushPull>>,
        hal::gpio::Pxx<hal::gpio::Output<hal::gpio::PushPull>>,
        hal::gpio::Pxx<hal::gpio::Output<hal::gpio::PushPull>>,
        hal::pwm::PwmChannel<pac::TIM1, hal::pwm::C4>,
        hal::pwm::PwmChannel<pac::TIM1, hal::pwm::C1>,
    >,
    output_l: i16,
    output_r: i16,
) {
    match (output_l.is_positive(), output_r.is_positive()) {
        (true, true) => {
            motor.a.forward();
            motor.a.set_duty((output_l + 200) as u16);
            motor.b.forward();
            motor.b.set_duty((output_r + 200) as u16);
        }
        (true, false) => {
            motor.a.forward();
            motor.a.set_duty((output_l + 200) as u16);
            motor.b.reverse();
            motor.b.set_duty(-(output_r - 200) as u16);
        }
        (false, true) => {
            motor.a.reverse();
            motor.a.set_duty(-(output_l - 200) as u16);
            motor.b.forward();
            motor.b.set_duty(output_r as u16);
        }
        (false, false) => {
            motor.a.reverse();
            motor.a.set_duty(-(output_l - 200) as u16);
            motor.b.reverse();
            motor.b.set_duty(-(output_r - 200) as u16);
        }
    }
}
