# Some code for a CH32V003 development board I bought on AliExpress to talk to 2 x Microchip MCP475 DACs via I2C
# as well as a Microchip RN4870 Bluetooth module via USART
#
# Ian Wraith (xxx)
#
# Pin Usage ..
#
# Port D0 - Debug LED
# Port D4 - USART CLK
# Port D5 - USART TX
# Port D6 - USART RX 
# Port C1 - I2C SDA
# Port C2 - I2C SCL 
# 
# This was turned into .bin file with the followed commands (your paths will be different) ..
#
# /opt/riscv/riscv32-unknown-linux-gnu/bin/as -march=rv32ec -mabi=ilp32e usart_and_i2c.s -o usart_and_i2c.o
# /opt/riscv/riscv32-unknown-linux-gnu/bin/objdump -S usart_and_i2c.o > usart_and_i2c.lst
# /opt/riscv/riscv32-unknown-linux-gnu/bin/objcopy -O binary usart_and_i2c.o usart_and_i2c.bin
# /opt/riscv/riscv32-unknown-linux-gnu/bin/objcopy -O ihex usart_and_i2c.o usart_and_i2c.hex
#
# Then sent to the dev board with ..
#
# minichlink -w usart_and_i2c.bin flash -b

.equ BIT00, 1
.equ BIT01, 2
.equ BIT02, 4
.equ BIT03, 8
.equ BIT04, 16
.equ BIT05, 32
.equ BIT06, 64
.equ BIT07, 128
.equ BIT08, 256
.equ BIT09, 512
.equ BIT10, 1024
.equ BIT11, 2048
 
.equ CTLR2_FREQ_RST, -31
.equ RCC_BASE,   0x40021
.equ FLASH_BASE, 0x40022
.equ GPIO_BASE,  0x40011
.equ MEMORY_BASE,0x20004
.equ USART_BASE, 0x40013

.equ DAC_ADDR1, 96
.equ DAC_ADDR2, 97

.global _start
.align 2	
.option norvc
.text

j startup
# Boilerplate stuff for the interupts which I'm not using
.word   0
.word   NMI_Handler               # NMI Handler                   
.word   HardFault_Handler         # Hard Fault Handler           
.word   0
.word   0
.word   0
.word   0
.word   0
.word   0
.word   0
.word   0
.word   SysTick_Handler           # SysTick Handler                 
.word   0
.word   SW_Handler                # SW Handler                      
.word   0
# External Interrupts                                         
.word   WWDG_IRQHandler           # Window Watchdog                 
.word   PVD_IRQHandler            # PVD through EXTI Line detect    
.word   FLASH_IRQHandler          # Flash                           
.word   RCC_IRQHandler            # RCC                             
.word   EXTI7_0_IRQHandler        # EXTI Line 7..0                  
.word   AWU_IRQHandler            # AWU                             
.word   DMA1_Channel1_IRQHandler  # DMA1 Channel 1                  
.word   DMA1_Channel2_IRQHandler  # DMA1 Channel 2                  
.word   DMA1_Channel3_IRQHandler  # DMA1 Channel 3                  
.word   DMA1_Channel4_IRQHandler  # DMA1 Channel 4                  
.word   DMA1_Channel5_IRQHandler  # DMA1 Channel 5                  
.word   DMA1_Channel6_IRQHandler  # DMA1 Channel 6                  
.word   DMA1_Channel7_IRQHandler  # DMA1 Channel 7                  
.word   ADC1_IRQHandler           # ADC1                            
.word   I2C1_EV_IRQHandler        # I2C1 Event                      
.word   I2C1_ER_IRQHandler        # I2C1 Error                      
.word   USART1_IRQHandler         # USART1                          
.word   SPI1_IRQHandler           # SPI1                            
.word   TIM1_BRK_IRQHandler       # TIM1 Break                      
.word   TIM1_UP_IRQHandler        # TIM1 Update                     
.word   TIM1_TRG_COM_IRQHandler   # TIM1 Trigger and Commutation    
.word   TIM1_CC_IRQHandler        # TIM1 Capture Compare            
.word   TIM2_IRQHandler           # TIM2                           

# Code catch any interupt that does occur
jal ra,diagnostic_led_off
icatcher_loop:
j icatcher_loop

# Standard setup boilerplate stuff
startup:
# Set the stack pointer to 0x20000700
lui sp,0x20000
addi sp,sp,0x700
li a0,128
csrw mstatus,a0
li a3,3
auipc a0,0x0
addi a0,a0,-182
or a0,a0,a3
csrw mtvec,a0
# 0xd6 is the address of main
li a5,0xd6
csrw mepc,a5
mret

# The main body of the code
main:
# Setup the clock (in this case using the external xtal)
# There are other functions here for external xtal with PLL and the internal clock (HCI)
jal ra,set_clock_external
# Setup the GPIOs
jal ra,set_gpio
# Setup I2C
jal ra,set_i2c
# Setup the USART
jal ra,set_usart

# Turn the diagnostic LED off
jal ra,diagnostic_led_off

# Send DAC1 half out
li a0,DAC_ADDR1
li a1,8
li a2,0
jal ra,i2c_send_data

# Send DAC2 to full out
li a0,DAC_ADDR2
li a1,15
li a2,255
jal ra,i2c_send_data

li a0,170
jal ra,send_usart_byte

jal ra,diagnostic_led_on

# Now the section of the program that will do stuff eventually
main_loop:

j main_loop

#####################################################################################################################################################

# Subroutines

# Store saved registers at the start of a subroutine
save_registers:
addi sp,sp,-12
sw ra,0(sp)
sw s0,4(sp)
sw s1,8(sp)
jr t0

# Restore saved registers and jump to whatever called a subroutine
restore_registers:
lw ra,0(sp)
lw s0,4(sp)
lw s1,8(sp)
addi sp,sp,12
ret

# Send I2C data to a MCP4725 DAC
# Register a0 contains the address of the DAC
# Register a1 contains the first byte sent to the DAC
# Register a2 contains the second byte sent to the DAC
#
# The data is sent in the following format (for a fast mode command to set the DAC register and not write to memory)
# First byte : Bits 7 to 1 are the device address (96 or 97) with bit 0 always 0
# Second byte : Bits 7 and 6 must be 0 , bits 5 and 4 are the PDx bits which must always be 0 , bits 3,2,1,0 are high 4 bits of the DAC value
# Third byte : This is the lower 8 bits of the DAC value
# So the DAC value is 12 bits
#
i2c_send_data:
jal t0,save_registers
# Set the first 20 bits of t2 to 0x40005 
lui t2,0x40005
# Loop if the BUSY bit is set
lui a3,0x20
busy_loop:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
beq a4,a3,busy_loop
# Set bit 8 (256) start in I2C1_CTLR1 
lw a3,1024(t2)
slli a3,a3,16
srli a3,a3,16
ori a3,a3,BIT08
# Writes this back to I2C1_CTLR1
sw a3,1024(t2)
# Loop until BUSY , MSL and SB bits are set (this is held in a3 the event mask)
# MSB 16 bits are STAR2 while LSB 16 bits are STAR1 
lui	a3,0x30
addi a3,a3,1
start_loop1:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
bne a4,a3,start_loop1
# Send the DAC address
slli a0,a0,1
andi a0,a0,254
sw a0,1040(t2)
# Loop while BUSY, MSL, ADDR, TXE and TRA flags are set (this is held in a3 the event mask)
lui	a3,0x70
addi a3,a3,0x82
addr_loop:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
bne a4,a3,addr_loop
# Send the 1st data byte
sw a1,1040(t2)
# Loop until TXE is set
li a3,128
first_byte_loop1:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
beq a4,zero,first_byte_loop1
# Send the 2nd data byte
sw a2,1040(t2)
# Loop until TXe is set
li a3,128
second_byte_loop1:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
beq a4,zero,second_byte_loop1
# Loop while TRA, BUSY, MSL, TXE and BTF flags are set (this is held in a3 the event mask)
lui	a3,0x70
addi a3,a3,0x84
stop_loop1:
# Combine STAR1 and STAR2 together
lhu a4,1044(t2)
slli t1,a4,16
srli t1,t1,16
lhu a4,1048(t2)
slli a4,a4,16
or a4,a4,t1
# AND with the event mask
and a4,a4,a3
bne a4,a3,stop_loop1
# Generate a stop state
# Set the STOP bit (bit 9)
# Load t1 with the contents of I2C1_CTLR1
lw a3,1024(t2)
or a3,a3,BIT09
# Writes this back to I2C1_CTLR1
sw a3,1024(t2)
j restore_registers


# Subroutine to take port D0 low thus turn the debug indicator LED on
diagnostic_led_on:
jal t0,save_registers
lui	a0,GPIO_BASE
lw t1,1040(a0)
lui t2,0x10
or t1,t1,t2
sw	t1,1040(a0)
j restore_registers

# Subroutine to take port D0 high thus turn the debug indicator LED off
diagnostic_led_off:
jal t0,save_registers
lui	a0,GPIO_BASE
lw t1,1040(a0)
# Write 1 to GPIOD_BSHR bit 0
ori t1,t1,BIT00
sw t1,1040(a0)
j restore_registers

# Subroutine to the turn on the internal clock (HSI)
set_clock_internal:
lui t1,RCC_BASE
# Setup RCC_CFGR0 (0x40021004) by ensuring it is 0
sw zero,4(t1)
# RCC_CTLR setup
# 00000000 10000 0 0 1
# HSITRIM 10000 , HSION
li a4,129
# Write the contents of a4 to t1 RCC_CTLR
sw a4,0(t1)
clk_loop1:	
# Sets the contents of t2 to the contents of RCC_CTRL
lw t2,0(t1)
# and with 2 so we just see the HSIRDY flag
andi t2,t2,2
# Loop if t2 contents are now zero
beq t2,zero,clk_loop1
clk_loop2:	
# Write the contents of RCC_CFGR0 to a4
lw a4,4(t1)
# AND with 12 to clear all bits except bits 2 and 3
andi a4,a4,12
# Loop if bits 2 and 3 aren't 0 so the system clock source is HSI
bne a4,zero,clk_loop2
ret

# Subroutine to the turn on the internal clock (HSI) with x 2 PLL
set_clock_internal_pll:
lui t1,RCC_BASE
# Setup RCC_CFGR0 (0x40021004) by ensuring it is 2 (PLL) + SYSCLCK divided by 2
# First bit 26 (67108864) to give SYSCLK on MCO pin
lui a4,16384
addi a4,a4,18
sw a4,4(t1)
# RCC_CTLR setup
# PLLON bit 24 (16777216)
lui a4,4096
# 00000000 10000 0 0 1
# HSITRIM 10000 , HSION
addi a4,a4,129
# Write the contents of a4 to t1 RCC_CTLR
sw a4,0(t1)
lui a0,8192
pclk_loop1:	
# Sets the contents of t2 to the contents of RCC_CTRL
lw t2,0(t1)
# and with 33554432 so we just see the PLLRDY flag
and t2,t2,a0
# Loop if t2 contents are now zero
beq t2,zero,pclk_loop1
li a3,8
pclk_loop2:	
# Write the contents of RCC_CFGR0 to a4
lw a4,4(t1)
# AND with 12 to clear all bits except bits 2 and 3
andi a4,a4,12
# Loop if bits 3 and 2 aren't 10 so the system clock source is HSI
bne a4,a3,pclk_loop2
ret

# Subroutine to the turn on the external clock (HSE)
set_clock_external:
lui a4,RCC_BASE
lui	a5,0x90
addi a5,a5,129
sw a5,0(a4)
lui	a3,0x20
hse_wait_loop1:
lw a5,0(a4)
and	a5,a5,a3
beqz a5,hse_wait_loop1
lui	a5,0x10
addi a5,a5,1
sw a5,4(a4)
li a3,4
lui a5,0x40021
hse_wait_loop2:
lw a4,4(a5)
andi a4,a4,12
bne	a4,a3,hse_wait_loop2
# Turn off the HSI
lui	a4,0x90
addi a4,a4,129
sw	a4,0(a5)
ret

# Subroutine to configure the GPIOs
set_gpio:
lui t1,RCC_BASE
# 0x40021018 (RCC_APB2PCENR)
# First enable USART1EN (bit 14)
lui t2,4
# then enable IOPCEN , IOPDEN and AFIOEN which is BIT04 + BIT05 + BIT00 = 49
addi t2,t2,49
# Write the modified data back to RCC_APB2PCENR
sw t2,24(t1)
# 0x4002101C (RCC_APB1PCENR)
# Set bit 21 I2C1EN to enable the I2C clock
lui t2,0x200
sw t2,28(t1)
lui t1,GPIO_BASE
# Port D configuration
# Port D0 - Debug LED CNF0 00 MODE0 01 universal push-pull at 10 MHz
# Port D1 - Programming (so leave untouched) at 01 00
# Port D4 - USART CK (set as push pull multiplex output 50 MHz) so CNF5 10 MODE5 11 
# Port D5 - USART TX (set as push pull multiplex output 50 MHz) so CNF5 10 MODE5 11 
# Port D6 - USART RX (set as floating input) so CNF6 01 MODE6 00
# So 0000 0100 1011 1011 0000 0000 0100 0001 (78643265)
#    7    6    5    4    3    2    1    0
lui t2,0x4bb0
addi t2,t2,65
# Write this value to GPIOD_CFGLR
sw	t2,1024(t1)
# Port C1 - I2C SDA
# Port C2 - I2C SCL 
# Port C1 configuration CNF1 and CNF2 to 11 (multiplexed open drain) MODE1 and MODE2 to 01 10 MHz
# Set to 221 shifted left by 4 bits = 3536 
li t0,221
slli t0,t0,4
sw t0,0(t1)
ret

# Subroutine to configure the I2C interface
set_i2c:
# Reset the I2C device
lui t1,RCC_BASE
lw a0,16(t1)
lui a3,0x200
or a0,a0,a3
sw a0,16(t1)
lui	a3,0xffe00
addi a3,a3,-1
lw a0,16(t1)
and a0,a0,a3
sw a0,16(t1)
# Now set things up
# Set the first 20 bits of a3 to 0x40005 
lui a3,0x40005
# Set I2C1_CTLR2 to 24 (MHz)
lw t1,1028(a3)
andi t1,t1,-64
ori t1,t1,24
sw t1,1028(a3)
# Set I2C1_CKCFGR to 24/0.2 (0.1 * 2) = 120 then double it for reasons unknown
li t1,240
sw t1,1052(a3)
# Set PE to enable I2C (bit 0) 
lw t1,1024(a3)
slli t1,t1,16
srli t1,t1,16
ori t1,t1,1
sw t1,1024(a3)
ret

# Subroutine to configure the USART interface
set_usart:
# Reset the USART device (bit 14 so 16384)
lui t1,RCC_BASE
lw a0,12(t1)
# Make a copy of a0 in t0
add t0,zero,a0
# Set bit 14
lui a3,4
or a0,a0,a3
sw a0,12(t1)
# Return to its previous value thus clearing bit 14
sw t0,12(t1)
# The RN4870 communications details are ..
# 115200 baud , 8 data bits , 1 stop bits and no flow control
lui t1,USART_BASE
# Add 0x800 to t1 to make 0x400138 (the USART base)
li t2,0x400
slli t2,t2,1
add t1,t1,t2
# The baud rate calculation is FCLK / (16 * USARTDIV)
# Where USARTDIV = DIV_M + (DIF_F / 16)
# Here FCLK is 24 MHz so 24000000
# USARTDIV is 13.0208
# DIV_M = 13
# DIV_F = 0
# is as close as we can get and actually gives us a baud rate of 115384
# So USART_BRR is 11010000 = 208
li t2,208
sw t2,8(t1)
# Enable the CLK pin via the CLKEN (bit 11) in USART_CTRL2
li t2,1
slli t2,t2,11
sw t2,16(t1)
# Enable the USART transmitter and receiver (bits 2 RE and 3 TE in USART_CTLR1) also UE (bit 13)
lui t2,2
addi t2,t2,12
sw t2,12(t1)
ret

# Send the 8 bits in a0 via the USART
send_usart_byte:
lui t1,USART_BASE
# Add 0x800 to t1 to make 0x400138 (the USART base)
li t2,0x400
slli t2,t2,1
add t1,t1,t2
# Put a0 into USART_DATAR
sw a0,4(t1)
# Loop while TXE is still zero
usart_tx_loop:
lw a1,0(t1)
# AND with BIT07
andi a1,a1,BIT07
# Branch back if a1 is zero
beqz a1,usart_tx_loop
ret

#####################################################################################################################################################


