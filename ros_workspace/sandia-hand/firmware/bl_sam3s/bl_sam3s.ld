OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
	rom (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00040000 /* flash, 256K */
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x0000c000 /* sram, 48K */
}

SECTIONS 
{ 
    .text 0x400000 :
    {
      . = ALIGN(4);
      _sfixed = .;
      KEEP(*bl_sam3s_startup.S.o(*.text))
      KEEP(*(.text*))  
      . = 0x1ff8;
      _sid = .;
      KEEP(*(.bl_hw_version))
      . = 0x1ffc;
      _sid = .;
      KEEP(*(.rs485_addr))
      . = 0x2000;
      KEEP(*(.bl_crush_start))
    } > rom 
    /*
    .text    0x402000 : 
    { 
        //. = ALIGN(4);
        _sfixed = .;
        KEEP(*(.vectors .vectors.*))
        *(.text .text.* .gnu.linkonce.t.*) 	      
        *(.glue_7t) *(.glue_7)		                
        *(.rodata .rodata* .gnu.linkonce.r.*)		    	                  
        *(.ARM.extab* .gnu.linkonce.armextab.*)

        . = ALIGN(4);
        KEEP(*(.init))
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(0x4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

        . = ALIGN(4);
        _efixed = .;            
    } > rom
    */

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    PROVIDE_HIDDEN (__exidx_start = .);
    .ARM.exidx :
    {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > rom
    PROVIDE_HIDDEN (__exidx_end = .);

    . = ALIGN(4); 
    _etext = .;

    .relocate : AT (_etext)
    {
        . = ALIGN(4);
        _srelocate = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _erelocate = .;
    } > ram

    . = ALIGN(4);
    .eh_frame :
    {
      KEEP(*(.eh_frame))
    } > ram

    .bss (NOLOAD) : 
    { 
        . = ALIGN(4);
        _sbss = . ;
        _szero = .;
        *(.debug_bss .debug_bss.*)
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = . ;
        _ezero = .;
    } > ram

    /* stack section */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack .stack.*)
    } > ram

    . = ALIGN(4); 
    _end = . ; 
}
