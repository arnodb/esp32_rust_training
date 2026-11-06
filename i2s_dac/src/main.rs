#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::Instant;
use esp_hal::dma::DmaError;
use esp_hal::dma_tx_stream_buffer;
use esp_hal::i2s::master::{Channels, DataFormat, I2s, TdmConfig};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use fixed::types::{I16F16, I32F32};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SAMPLE_RATE_HZ: u32 = 44100;

const DATA_FORMAT: DataFormat = DataFormat::Data16Channel16;

const WIDTH: usize = match DATA_FORMAT {
    DataFormat::Data16Channel16 => 4,
    DataFormat::Data32Channel32 => 8,
    _ => todo!(),
};

const MASK: usize = WIDTH - 1;

const MIN_FREQUENCY: u32 = 220;
const MAX_FREQUENCY: u32 = 660;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.4.0
    // generator parameters: -o esp32c6 -o esp32c6-wroom-1 -o unstable-hal -o embassy -o stable-x86_64-unknown-linux-gnu

    let config = esp_hal::Config::default();
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // TODO: Spawn some tasks
    let _ = spawner;

    let i2s = I2s::new(
        peripherals.I2S0,
        peripherals.DMA_CH0,
        TdmConfig::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(SAMPLE_RATE_HZ))
            .with_data_format(DATA_FORMAT)
            .with_channels(Channels::STEREO),
    )
    .unwrap()
    .into_async();

    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO18)
        .with_ws(peripherals.GPIO19)
        .with_dout(peripherals.GPIO20)
        .build();

    let mut audio_data = [0u8; (SAMPLE_RATE_HZ as usize / 16 * WIDTH) & !MASK];
    let mut frequency = 440;
    let mut freq_up = true;
    let mut period_pos = 0;
    let shape = Shape::Sine;
    period_pos = sample_more(frequency, shape, period_pos, &mut audio_data);

    let mut tx_buffer = dma_tx_stream_buffer!(4 * 4092, 1024);
    let mut pos = 0;
    let mut transfer = i2s_tx.write(tx_buffer).unwrap();

    let mut last_frequency_change = Instant::now().as_millis();

    loop {
        if let Err(DmaError::DescriptorError) = transfer.wait_for_available_async().await {
            println!("descriptor error");
            (i2s_tx, tx_buffer) = transfer.stop();
            transfer = i2s_tx.write(tx_buffer).unwrap();
        }
        let mut avail = transfer.available_bytes();
        while avail >= WIDTH {
            let count = avail.min(audio_data.len() - pos) & !MASK;
            let pushed = transfer.push(&audio_data[pos..pos + count]);
            pos = (pos + pushed) % audio_data.len();
            if pos == 0 {
                period_pos = sample_more(frequency, shape, period_pos, &mut audio_data);
            }
            let now = Instant::now().as_millis();
            if now / 10 > last_frequency_change / 10 {
                let new_frequency = if freq_up {
                    frequency + 1
                } else {
                    frequency - 1
                };
                frequency = if new_frequency > MAX_FREQUENCY {
                    freq_up = false;
                    MAX_FREQUENCY
                } else if new_frequency < MIN_FREQUENCY {
                    freq_up = true;
                    MIN_FREQUENCY
                } else {
                    new_frequency
                };
                last_frequency_change = now;
            }
            avail -= count;
        }
    }
}

const SCALE: u64 = 32768;

#[derive(Clone, Copy)]
enum Shape {
    #[allow(unused)]
    Square,
    #[allow(unused)]
    Sine,
    #[allow(unused)]
    Max,
}

fn sample_more(frequency: u32, shape: Shape, period_pos: u64, audio_data: &mut [u8]) -> u64 {
    //let start = Instant::now();

    for (i, chunk) in audio_data.chunks_mut(WIDTH).enumerate() {
        let pos =
            (period_pos + frequency as u64 * SCALE * i as u64 / SAMPLE_RATE_HZ as u64) % SCALE;
        let sample_value = match shape {
            Shape::Square => {
                if pos >= SCALE / 2 {
                    6000_i16
                } else {
                    -6000_i16
                }
            }
            Shape::Sine => {
                let value = I16F16::from_num(6000)
                    * cordic::sin(
                        (I32F32::from_num(pos * 2) * I32F32::PI / I32F32::from_num(SCALE))
                            .to_num::<I16F16>(),
                    );
                value.to_num::<i16>()
            }
            Shape::Max => i16::MAX,
        };
        let bytes = sample_value.to_le_bytes();
        match DATA_FORMAT {
            DataFormat::Data16Channel16 => {
                // Left
                chunk[0] = bytes[0];
                chunk[1] = bytes[1];
                // Right
                chunk[2] = bytes[0];
                chunk[3] = bytes[1];
            }
            DataFormat::Data32Channel32 => {
                // Left
                chunk[0] = 0;
                chunk[1] = 0;
                chunk[2] = bytes[0];
                chunk[3] = bytes[1];
                // Right
                chunk[4] = 0;
                chunk[5] = 0;
                chunk[6] = bytes[0];
                chunk[7] = bytes[1];
            }
            _ => todo!(),
        }
    }

    /*
    let end = Instant::now();
    println!(
        "took {} to generate {} samples",
        end - start,
        audio_data.len() / WIDTH
    );
    */

    (period_pos + frequency as u64 * SCALE * (audio_data.len() as u64 / 4) / SAMPLE_RATE_HZ as u64)
        % SCALE
}
