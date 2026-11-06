#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use esp_hal::dma::DmaError;
use esp_hal::i2s::master::{Channels, DataFormat, I2s, TdmConfig};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{dma_rx_stream_buffer, dma_tx_stream_buffer};
use esp_println::println;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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

    let i2s0 = I2s::new(
        peripherals.I2S0,
        peripherals.DMA_CH0,
        TdmConfig::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(SAMPLE_RATE_HZ))
            .with_data_format(DataFormat::Data32Channel32)
            .with_channels(Channels::STEREO)
            .with_signal_loopback(true),
    )
    .unwrap()
    .into_async();

    let mut i2s_tx = i2s0
        .i2s_tx
        .with_bclk(peripherals.GPIO18)
        .with_ws(peripherals.GPIO19)
        .with_dout(peripherals.GPIO20)
        .build();

    let mut i2s_rx = i2s0.i2s_rx.with_din(peripherals.GPIO21).build();

    const MASK: usize = 7;

    let mut audio_data = [0u8; (SAMPLE_RATE_HZ as usize / 16 * 8) & !MASK];

    let mut tx_buffer = dma_tx_stream_buffer!(4 * 4092, 1024);
    let mut tx_transfer = i2s_tx.write(tx_buffer).unwrap();

    let mut rx_buffer = dma_rx_stream_buffer!(4 * 4092, 1024);
    let mut rx_transfer = i2s_rx.read(rx_buffer).unwrap();

    loop {
        match rx_transfer.wait_for_available_async().await {
            Ok(()) => {}
            Err(DmaError::DescriptorError) => {
                println!("rx descriptor error");
                (i2s_rx, rx_buffer) = rx_transfer.stop();
                rx_transfer = i2s_rx.read(rx_buffer).unwrap();
            }
            Err(err) => {
                println!("rx error, {err:?}");
            }
        }
        let avail = rx_transfer.available_bytes();
        let count = avail.min(audio_data.len()) & !MASK;
        let popped = rx_transfer.pop(&mut audio_data[0..count]);
        let mut remaining = popped;
        {
            for chunk in audio_data[0..popped].as_chunks_mut::<8>().0 {
                let left_raw = i32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                chunk[4..8].copy_from_slice(&(left_raw).to_le_bytes());
            }
        }
        loop {
            if remaining == 0 {
                break;
            }
            match tx_transfer.wait_for_available_async().await {
                Ok(()) => {}
                Err(DmaError::DescriptorError) => {
                    println!("tx descriptor error");
                    (i2s_tx, tx_buffer) = tx_transfer.stop();
                    tx_transfer = i2s_tx.write(tx_buffer).unwrap();
                }
                Err(err) => {
                    println!("tx error, {err:?}");
                }
            }
            let avail = tx_transfer.available_bytes();
            let count = remaining.min(avail) & !MASK;
            let pushed =
                tx_transfer.push(&audio_data[popped - remaining..popped - remaining + count]);
            remaining -= pushed;
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}

const SAMPLE_RATE_HZ: u32 = 44100;
