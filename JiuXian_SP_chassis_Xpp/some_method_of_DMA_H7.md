# 一些针对达妙开发板的DMA发送不了的解（歪）决（门）办（邪）法（道）
## 贡献人：庆园阁 陈光钊（2026赛季、2027赛季）

### 如果发现达妙开发板的DMA发送不了，可以尝试以下方法来绕开Cache设置：
### 去STM32H723XG_FLASH.ld文件中找到以下代码：
```
  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)

      . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
      PROVIDE( __bss_end = .);
  } >DTCMRAM
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );

  PROVIDE( __bss_start = __tbss_start );
  PROVIDE( __bss_size = __bss_end - __bss_start );

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM
```
### 将其修改为：
```
  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)

      . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
      PROVIDE( __bss_end = .);
  } >DTCMRAM
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );

  PROVIDE( __bss_start = __tbss_start );
  PROVIDE( __bss_size = __bss_end - __bss_start );

  /* 这里加入代码 */
  /* DMA Buffers Section (AXI SRAM - Accessible by DMA1/DMA2) */   
  .dma_buffer (NOLOAD) : ALIGN(32)
  {
    *(.dma_buffer*)
  } >RAM_D1
  /* 以上为加入部分 */

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM
```
### 然后，跑回你想要发送的数据前加入如下代码（看我划线部分！！！）：
```
    __attribute__((section(".dma_buffer"))) uint8_t the_data_you_want_to_send[YOUR_DATA_SIZE];
    ………………………………………………………………………………………………………
```
### 最后再正常发送即可！！！   ：-）

## 问题剖析
### 事实上测试的时候发现轮询发送和IT发送都没有问题但是DMA发送出了问题，基本原因就是：
### 问题的核心在于 物理地址的解码权限（Address Decoding Permissions）。虽然 DTCM 和 AXI SRAM 都是物理 RAM，但它们在ARM v7-M架构的内存映射表中隶属于完全不同的总线域，导致 DMA 控制器无法对特定地址区间进行译码。
|内存区域 | 起始地址 | 终止地址 | 总线归属 | DMA1/DMA2访问权限 |
|---------|----------|----------|----------|------------------|
| DTCM RAM | 0x20000000 | 0x2001FFFF | DTCM总线 | 无法访问 |
| AXI SRAM | 0x24000000 | 0x2401FFFF | AXI总线 | 可访问 |
| SRAM1 | 0x30000000 | 0x3001FFFF | AHB Bus Matrix | 可访问 |
| SRAM2 | 0x30020000 | 0x3003FFFF | AHB Bus Matrix | 可访问 |
### 从地址角度看，这是 0x2000_xxxx (TCM Private) 与 0x2400_xxxx (AXI Shared)的区别。DMA 只能访问系统共享域的地址，无法跨越总线矩阵访问内核私有域的地址。

