//! BitaxeBonanza mining board support.
//!
//! The BitaxeBonanza board is a mining board with 4 BZM2 ASIC chips, communicating via
//! USB using two serial ports: a control UART for GPIO/I2C and a data UART for
//! ASIC communication with 8-bit to 9-bit serial translation.

use async_trait::async_trait;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::watch;
use tokio::time::{Duration, sleep};
use tokio_serial::SerialPortBuilderExt;
use tokio_util::codec::{FramedRead, FramedWrite};

use super::{
    Board, BoardDescriptor, BoardError, BoardInfo,
    pattern::{BoardPattern, Match, StringMatch},
};
use crate::{
    asic::{
        bzm2::{FrameCodec, smoke, thread::Bzm2Thread},
        hash_thread::{AsicEnable, BoardPeripherals, HashThread, ThreadRemovalSignal},
    },
    error::Error,
    transport::{
        UsbDeviceInfo,
        serial::{SerialControl, SerialReader, SerialStream, SerialWriter},
    },
};

const BITAXE_BONANZA_VID: u16 = 0xc0de;
const BITAXE_BONANZA_PID: u16 = 0xcafe;
const BITAXE_BONANZA_MANUFACTURER: &str = "OSMU";
const BITAXE_BONANZA_PRODUCT: &str = "BitaxeBonanza";

/// Number of BZM2 ASICs on a BitaxeBonanza board.
const ASICS_PER_BOARD: usize = 4;

/// Default baud rate for the BitaxeBonanza data UART (5 Mbps).
const DATA_UART_BAUD: u32 = 5_000_000;

/// Baud rate for the BitaxeBonanza control UART.
const CONTROL_UART_BAUD: u32 = 115_200;

/// BitaxeBonanza control GPIO: 5V power enable.
const GPIO_5V_EN: u8 = 0x01;
/// BitaxeBonanza control GPIO: ASIC reset (active-low).
const GPIO_ASIC_RST: u8 = 0x02;
/// BitaxeBonanza control GPIO: ASIC trip status.
#[allow(dead_code)]
const GPIO_ASIC_TRIP: u8 = 0x03;
/// BitaxeBonanza control GPIO: VR power enable.
const GPIO_VR_EN: u8 = 0x04;
/// BitaxeBonanza control GPIO: VR power-good status.
const GPIO_VR_PGOOD: u8 = 0x05;
/// Control request ID used for GPIO operations. The firmware echoes this in responses.
const CTRL_ID_GPIO: u8 = 0xAB;
/// Control protocol page for GPIO.
const CTRL_PAGE_GPIO: u8 = 0x06;

fn format_hex(data: &[u8]) -> String {
    data.iter()
        .map(|byte| format!("{:02X}", byte))
        .collect::<Vec<_>>()
        .join(" ")
}

/// BitaxeBonanza mining board.
pub struct BitaxeBonanzaBoard {
    device_info: UsbDeviceInfo,
    control_port: Option<String>,
    data_reader: Option<FramedRead<SerialReader, FrameCodec>>,
    data_writer: Option<FramedWrite<SerialWriter, FrameCodec>>,
    data_control: Option<SerialControl>,
    thread_shutdown: Option<watch::Sender<ThreadRemovalSignal>>,
}

impl BitaxeBonanzaBoard {
    /// Create a new BitaxeBonanza board instance.
    pub fn new(device_info: UsbDeviceInfo) -> Result<Self, BoardError> {
        Ok(Self {
            device_info,
            control_port: None,
            data_reader: None,
            data_writer: None,
            data_control: None,
            thread_shutdown: None,
        })
    }

    /// Early bring-up init path.
    ///
    /// Until full thread integration lands, we run a basic UART smoke test
    /// (NOOP + READREG ASIC_ID) during board initialization.
    pub async fn initialize(&mut self) -> Result<(), BoardError> {
        let (control_port, data_port) = {
            let serial_ports = self.device_info.serial_ports().map_err(|e| {
                BoardError::InitializationFailed(format!("Failed to enumerate serial ports: {}", e))
            })?;

            if serial_ports.len() != 2 {
                return Err(BoardError::InitializationFailed(format!(
                    "BitaxeBonanza requires exactly 2 serial ports, found {}",
                    serial_ports.len()
                )));
            }

            (serial_ports[0].clone(), serial_ports[1].clone())
        };

        tracing::info!(
            serial = ?self.device_info.serial_number,
            control_port = %control_port,
            data_port = %data_port,
            data_baud = DATA_UART_BAUD,
            control_baud = CONTROL_UART_BAUD,
            asics = ASICS_PER_BOARD,
            "Running BitaxeBonanza ASIC smoke test during initialization"
        );

        // Match known-good BitaxeBonanza bring-up sequence:
        // 1) VR off and settle
        // 2) Enable 5V rail
        // 3) Enable VR
        // 4) Pulse ASIC reset low/high
        // 5) Wait for UART startup
        self.bringup_power_and_reset(&control_port).await?;
        self.control_port = Some(control_port);

        let result = smoke::run_smoke(&data_port, 0).await.map_err(|e| {
            BoardError::InitializationFailed(format!(
                "BitaxeBonanza ASIC smoke test failed: {:#}",
                e
            ))
        })?;

        tracing::info!(
            logical_asic = result.logical_asic,
            asic_hw_id = result.asic_hw_id,
            asic_id = format_args!("0x{:08x}", result.asic_id),
            "BitaxeBonanza ASIC smoke test succeeded"
        );

        let data_stream = SerialStream::new(&data_port, DATA_UART_BAUD).map_err(|e| {
            BoardError::InitializationFailed(format!(
                "Failed to open BitaxeBonanza data port: {}",
                e
            ))
        })?;
        let (data_reader, data_writer, data_control) = data_stream.split();
        self.data_reader = Some(FramedRead::new(data_reader, FrameCodec::default()));
        self.data_writer = Some(FramedWrite::new(data_writer, FrameCodec::default()));
        self.data_control = Some(data_control);

        Ok(())
    }

    async fn bringup_power_and_reset(&self, control_port: &str) -> Result<(), BoardError> {
        let mut control_stream = tokio_serial::new(control_port, CONTROL_UART_BAUD)
            .open_native_async()
            .map_err(|e| {
                BoardError::InitializationFailed(format!(
                    "Failed to open BitaxeBonanza control port {}: {}",
                    control_port, e
                ))
            })?;

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_VR_EN, false).await?;
        sleep(Duration::from_millis(2000)).await;

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_5V_EN, true).await?;
        sleep(Duration::from_millis(100)).await;

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_VR_EN, true).await?;
        sleep(Duration::from_millis(100)).await;

        let vr_pgood =
            Self::control_gpio_read(&mut control_stream, CTRL_ID_GPIO, GPIO_VR_PGOOD).await?;
        if !vr_pgood {
            return Err(BoardError::HardwareControl(
                "BitaxeBonanza VR_PGOOD did not assert after enabling VR".into(),
            ));
        }

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_ASIC_RST, false).await?;
        sleep(Duration::from_millis(100)).await;

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_ASIC_RST, true).await?;
        sleep(Duration::from_millis(1000)).await;

        Ok(())
    }

    async fn control_gpio_write(
        stream: &mut tokio_serial::SerialStream,
        request_id: u8,
        pin: u8,
        value_high: bool,
    ) -> Result<(), BoardError> {
        // Packet format: [len:u16_le][id][bus][page][cmd=pin][value].
        let packet: [u8; 7] = [
            0x07,
            0x00,
            request_id,
            0x00,
            CTRL_PAGE_GPIO,
            pin,
            if value_high { 0x01 } else { 0x00 },
        ];
        tracing::debug!(
            request_id = format_args!("0x{:02X}", request_id),
            pin,
            value = if value_high { 1 } else { 0 },
            tx = %format_hex(&packet),
            "BitaxeBonanza ctrl gpio tx"
        );
        stream.write_all(&packet).await.map_err(|e| {
            BoardError::HardwareControl(format!(
                "Failed to write GPIO control packet (pin {}): {}",
                pin, e
            ))
        })?;

        // Ack is 4 bytes. Byte[2] should echo the request ID.
        let mut ack = [0u8; 4];
        stream.read_exact(&mut ack).await.map_err(|e| {
            BoardError::HardwareControl(format!(
                "Failed to read GPIO control ack (pin {}): {}",
                pin, e
            ))
        })?;
        tracing::debug!(
            request_id = format_args!("0x{:02X}", request_id),
            pin,
            rx = %format_hex(&ack),
            "BitaxeBonanza ctrl gpio rx"
        );
        if ack[2] != request_id {
            return Err(BoardError::HardwareControl(format!(
                "GPIO ack ID mismatch for pin {}: expected 0x{:02x}, got 0x{:02x}",
                pin, request_id, ack[2]
            )));
        }

        if ack[3] != u8::from(value_high) {
            return Err(BoardError::HardwareControl(format!(
                "GPIO ack value mismatch for pin {}: expected {}, got {}",
                pin,
                u8::from(value_high),
                ack[3]
            )));
        }

        Ok(())
    }

    async fn control_gpio_read(
        stream: &mut tokio_serial::SerialStream,
        request_id: u8,
        pin: u8,
    ) -> Result<bool, BoardError> {
        // Packet format: [len:u16_le][id][bus][page][cmd=pin].
        let packet: [u8; 6] = [0x06, 0x00, request_id, 0x00, CTRL_PAGE_GPIO, pin];
        tracing::debug!(
            request_id = format_args!("0x{:02X}", request_id),
            pin,
            tx = %format_hex(&packet),
            "BitaxeBonanza ctrl gpio read tx"
        );
        stream.write_all(&packet).await.map_err(|e| {
            BoardError::HardwareControl(format!(
                "Failed to write GPIO read packet (pin {}): {}",
                pin, e
            ))
        })?;

        let mut ack = [0u8; 4];
        stream.read_exact(&mut ack).await.map_err(|e| {
            BoardError::HardwareControl(format!(
                "Failed to read GPIO read response (pin {}): {}",
                pin, e
            ))
        })?;
        tracing::debug!(
            request_id = format_args!("0x{:02X}", request_id),
            pin,
            rx = %format_hex(&ack),
            "BitaxeBonanza ctrl gpio read rx"
        );
        if ack[2] != request_id {
            return Err(BoardError::HardwareControl(format!(
                "GPIO read response ID mismatch for pin {}: expected 0x{:02x}, got 0x{:02x}",
                pin, request_id, ack[2]
            )));
        }

        Ok(ack[3] != 0)
    }

    async fn hold_in_reset(&self) -> Result<(), BoardError> {
        let control_port = self.control_port.as_ref().ok_or_else(|| {
            BoardError::InitializationFailed("BitaxeBonanza control port not initialized".into())
        })?;

        let mut control_stream = tokio_serial::new(control_port, CONTROL_UART_BAUD)
            .open_native_async()
            .map_err(|e| {
                BoardError::InitializationFailed(format!(
                    "Failed to open BitaxeBonanza control port {}: {}",
                    control_port, e
                ))
            })?;

        Self::control_gpio_write(&mut control_stream, CTRL_ID_GPIO, GPIO_ASIC_RST, false).await
    }
}

struct BitaxeBonanzaAsicEnable {
    control_port: String,
}

#[async_trait]
impl AsicEnable for BitaxeBonanzaAsicEnable {
    async fn enable(&mut self) -> anyhow::Result<()> {
        let mut control_stream = tokio_serial::new(&self.control_port, CONTROL_UART_BAUD)
            .open_native_async()
            .map_err(|e| anyhow::anyhow!("failed to open control port: {}", e))?;
        BitaxeBonanzaBoard::control_gpio_write(
            &mut control_stream,
            CTRL_ID_GPIO,
            GPIO_ASIC_RST,
            true,
        )
        .await
        .map_err(|e| anyhow::anyhow!("failed to release BZM2 reset: {}", e))
    }

    async fn disable(&mut self) -> anyhow::Result<()> {
        let mut control_stream = tokio_serial::new(&self.control_port, CONTROL_UART_BAUD)
            .open_native_async()
            .map_err(|e| anyhow::anyhow!("failed to open control port: {}", e))?;
        BitaxeBonanzaBoard::control_gpio_write(
            &mut control_stream,
            CTRL_ID_GPIO,
            GPIO_ASIC_RST,
            false,
        )
        .await
        .map_err(|e| anyhow::anyhow!("failed to assert BZM2 reset: {}", e))
    }
}

#[async_trait]
impl Board for BitaxeBonanzaBoard {
    fn board_info(&self) -> BoardInfo {
        BoardInfo {
            model: BITAXE_BONANZA_PRODUCT.to_string(),
            firmware_version: None,
            serial_number: self.device_info.serial_number.clone(),
        }
    }

    async fn shutdown(&mut self) -> Result<(), BoardError> {
        if let Some(ref tx) = self.thread_shutdown {
            if let Err(e) = tx.send(ThreadRemovalSignal::Shutdown) {
                tracing::warn!(
                    "Failed to send shutdown signal to BitaxeBonanza thread: {}",
                    e
                );
            }
        }

        self.hold_in_reset().await?;
        Ok(())
    }

    async fn create_hash_threads(&mut self) -> Result<Vec<Box<dyn HashThread>>, BoardError> {
        let (removal_tx, removal_rx) = watch::channel(ThreadRemovalSignal::Running);
        self.thread_shutdown = Some(removal_tx);

        let data_reader = self
            .data_reader
            .take()
            .ok_or(BoardError::InitializationFailed(
                "No BitaxeBonanza data reader available".into(),
            ))?;
        let data_writer = self
            .data_writer
            .take()
            .ok_or(BoardError::InitializationFailed(
                "No BitaxeBonanza data writer available".into(),
            ))?;

        let control_port = self
            .control_port
            .clone()
            .ok_or(BoardError::InitializationFailed(
                "No BitaxeBonanza control port available".into(),
            ))?;
        let asic_enable = BitaxeBonanzaAsicEnable { control_port };
        let peripherals = BoardPeripherals {
            asic_enable: Some(Box::new(asic_enable)),
            voltage_regulator: None,
        };

        let thread_name = match &self.device_info.serial_number {
            Some(serial) => format!("BitaxeBonanza-{}", &serial[..8.min(serial.len())]),
            None => BITAXE_BONANZA_PRODUCT.to_string(),
        };

        let thread = Bzm2Thread::new(
            thread_name,
            data_reader,
            data_writer,
            peripherals,
            removal_rx,
            ASICS_PER_BOARD as u8,
        );
        Ok(vec![Box::new(thread)])
    }
}

// Factory function to create a BitaxeBonanza board from USB device info
async fn create_from_usb(device: UsbDeviceInfo) -> crate::error::Result<Box<dyn Board + Send>> {
    let mut board = BitaxeBonanzaBoard::new(device)
        .map_err(|e| Error::Hardware(format!("Failed to create board: {}", e)))?;

    board
        .initialize()
        .await
        .map_err(|e| Error::Hardware(format!("Failed to initialize BitaxeBonanza board: {}", e)))?;

    Ok(Box::new(board))
}

// Register this board type with the inventory system
inventory::submit! {
    BoardDescriptor {
        pattern: BoardPattern {
            vid: Match::Specific(BITAXE_BONANZA_VID),
            pid: Match::Specific(BITAXE_BONANZA_PID),
            manufacturer: Match::Specific(StringMatch::Exact(BITAXE_BONANZA_MANUFACTURER)),
            product: Match::Specific(StringMatch::Exact(BITAXE_BONANZA_PRODUCT)),
            serial_pattern: Match::Any,
        },
        name: BITAXE_BONANZA_PRODUCT,
        create_fn: |device| Box::pin(create_from_usb(device)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_creation() {
        let device = UsbDeviceInfo::new_for_test(
            0xc0de,
            0xcafe,
            Some("TEST001".to_string()),
            Some(BITAXE_BONANZA_MANUFACTURER.to_string()),
            Some(BITAXE_BONANZA_PRODUCT.to_string()),
            "/sys/devices/test".to_string(),
        );

        let board = BitaxeBonanzaBoard::new(device);
        assert!(board.is_ok());

        let board = board.unwrap();
        assert_eq!(board.board_info().model, BITAXE_BONANZA_PRODUCT);
    }

    #[test]
    fn test_bitaxe_bonanza_usb_pattern() {
        let pattern = BoardPattern {
            vid: Match::Specific(BITAXE_BONANZA_VID),
            pid: Match::Specific(BITAXE_BONANZA_PID),
            manufacturer: Match::Specific(StringMatch::Exact(BITAXE_BONANZA_MANUFACTURER)),
            product: Match::Specific(StringMatch::Exact(BITAXE_BONANZA_PRODUCT)),
            serial_pattern: Match::Any,
        };

        let device = UsbDeviceInfo::new_for_test(
            BITAXE_BONANZA_VID,
            BITAXE_BONANZA_PID,
            Some("92eed1c0".to_string()),
            Some(BITAXE_BONANZA_MANUFACTURER.to_string()),
            Some(BITAXE_BONANZA_PRODUCT.to_string()),
            "/sys/devices/test".to_string(),
        );
        assert!(pattern.matches(&device));

        let unsupported_product = UsbDeviceInfo::new_for_test(
            BITAXE_BONANZA_VID,
            BITAXE_BONANZA_PID,
            Some("92eed1c0".to_string()),
            Some(BITAXE_BONANZA_MANUFACTURER.to_string()),
            Some("UnsupportedBzm2Board".to_string()),
            "/sys/devices/test".to_string(),
        );
        assert!(!pattern.matches(&unsupported_product));
    }

    #[test]
    fn test_bitaxe_bonanza_gpio_protocol_numbers() {
        assert_eq!(GPIO_5V_EN, 0x01);
        assert_eq!(GPIO_ASIC_RST, 0x02);
        assert_eq!(GPIO_ASIC_TRIP, 0x03);
        assert_eq!(GPIO_VR_EN, 0x04);
        assert_eq!(GPIO_VR_PGOOD, 0x05);
    }
}
