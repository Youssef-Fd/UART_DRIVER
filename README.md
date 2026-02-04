
# UART Driver for STM32F429 (Bare-Metal)

## Project Overview

This project implements a **fully custom UART driver** for the **STM32F429 microcontroller**, written at the **register level** without using STM32Cube HAL.

The driver supports interrupt-driven transmission and reception and demonstrates how to build reusable embedded drivers with proper state machines and error handling.

The goal is to deeply understand:

* UART peripheral internals
* Interrupt-driven communication
* NVIC configuration
* Driver abstraction layers
* Bare-metal embedded software architecture

---

## Key Learning Objectives

* Direct register-level programming (bare-metal)
* Writing reusable HAL-like drivers
* Interrupt-driven UART communication
* NVIC interrupt handling
* Error detection and recovery
* TX/RX state machine design
* Modular embedded architecture

---

## Hardware & Tools

* **Microcontroller**: STM32F429ZIT6 (STM32F4 series)
* **IDE**: STM32CubeIDE
* **Language**: C
* **Communication**: UART (USART1 default)
* **Terminal**: PuTTY / TeraTerm / minicom
* **Clock**: 42 MHz APB2 peripheral clock

---


## Driver Architecture

The UART driver is built around a **handle structure** that stores:

* Peripheral instance pointer
* TX/RX buffers
* Transfer counters
* Driver state
* Error codes

Example handle:

```c
uart_handle_t huart1;
````

The driver separates:

* Hardware configuration
* Interrupt management
* TX/RX logic
* Error handling
* Application interface

---

## UART Configuration

Default UART configuration:

* Baud rate: 9600
* Word length: 8 bits
* Stop bits: 1
* Parity: None
* Oversampling: 16x
* Mode: TX + RX
* Interrupt-driven communication

Pins:

* **PA9**  → USART1_TX
* **PA10** → USART1_RX

Alternate Function: AF7

---

## Main Driver APIs

```c
void hal_uart_init(uart_handle_t *huart);
void hal_uart_tx(uart_handle_t *huart, uint8_t *buffer, uint32_t len);
void hal_uart_rx(uart_handle_t *huart, uint8_t *buffer, uint32_t len);
void hal_uart_handle_interrupt(uart_handle_t *huart);
```

---

## Interrupt System Overview

The UART uses **NVIC-based interrupts** to handle TX and RX events.

Each USART has a fixed IRQ entry in the vector table:

| Peripheral | IRQ Number | ISR Name          |
| ---------- | ---------- | ----------------- |
| USART1     | 37         | USART1_IRQHandler |
| USART2     | 38         | USART2_IRQHandler |
| USART3     | 39         | USART3_IRQHandler |

Example ISR:

```c
void USART1_IRQHandler(void)
{
    hal_uart_handle_interrupt(&huart1);
}
```

The ISR only forwards execution to the driver.

---

## Interrupt Types Handled

### RXNE – Receive Not Empty

Triggered when new data arrives.

Behavior:

* Reads DR register
* Stores byte in buffer
* Updates RX counter
* Calls callback when complete

---

### TXE – Transmit Data Register Empty

Triggered when UART is ready for the next byte.

Behavior:

* Loads next byte into DR
* Advances buffer pointer
* When last byte is sent:

  * Disable TXE interrupt
  * Enable TC interrupt

---

### TC – Transmission Complete

Triggered when the full frame is transmitted.

Behavior:

* Disable TC interrupt
* Reset TX state
* Call TX completion callback

---

### Error Interrupts

Handled errors:

* Parity Error (PE)
* Framing Error (FE)
* Noise Error (NE)
* Overrun Error (ORE)

Flags cleared by:

```c
volatile uint32_t tmp;
tmp = USARTx->SR;
tmp = USARTx->DR;
```

Driver accumulates error codes and resets state machines.

---

## TX State Machine

```
READY
  |
  +-- hal_uart_tx()
  v
BUSY_TX
  |
  +-- TXE interrupts send bytes
  |
  +-- Last byte → enable TC
  v
WAIT_TC
  |
  +-- TC interrupt
  v
READY
```

---

## RX State Machine

```
READY
  |
  +-- hal_uart_rx()
  v
BUSY_RX
  |
  +-- RXNE interrupts receive bytes
  |
  +-- Last byte received
  v
READY
```

---

## Main Application Flow

1. Initialize GPIO and UART
2. Configure NVIC interrupt
3. Start RX or TX transfer
4. Interrupt handler manages communication
5. Application receives callback when done

---

## Register Quick Reference

### USART Registers

* **SR** – Status Register

  * TXE bit 7
  * TC bit 6
  * RXNE bit 5
* **DR** – Data Register
* **BRR** – Baud Rate Register
* **CR1** – Enable + interrupt bits
* **CR2** – Stop bits
* **CR3** – Error interrupts

---

## Timing Characteristics

At 9600 baud:

* 1 bit = 104 µs
* 1 frame (10 bits) ≈ 1.04 ms
* 100 bytes ≈ 104 ms

Interrupt latency is negligible compared to frame time.

---

## Debugging Tips

### No Serial Output

* Verify USART clock enabled
* Check AF7 configuration on PA9/PA10
* Confirm BRR value matches baud rate
* Ensure NVIC interrupt enabled

### Garbled Data

* Baud rate mismatch
* Wrong clock configuration
* Missing stop bits configuration

### Interrupt Not Triggering

* NVIC priority not set
* Global interrupts disabled
* Wrong ISR name

---

## Expected Behavior

The driver supports:

* Continuous TX transfers
* Continuous RX transfers
* Error detection and recovery
* Non-blocking communication

Terminal output should be stable and loss-free.

---

## References

* STM32F429 Reference Manual (RM0090)
* STM32F4 Datasheet
* ARM Cortex-M4 User Guide
* ST Application Notes on USART

---
