	extern main
_start:
	call main
	sgdt [gdt_ptr]
	mov eax,stack_ring3_len-1
	mov ebx,[gdt_ptr+0x2];ebx存储gdt的基址
	sub ebx,0xc0000000
	mov [ebx+0x28+0],ax;ebx+0x28为ring3的栈的gdt，+0表示ring3的基址
	mov eax,stack_ring3
	mov [ebx+0x28+2],ax
	shr eax,16
	mov [ebx+0x28+7],ah
	mov [ebx+0x28+4],al
	push 0x2b
	push top_of_stack_ring3
	push 0x23
	push _ring3_test
	;jmp $
	;sti
	retf

stack_ring3:
	times 100 db 0
	top_of_stack_ring3	equ	$-stack_ring3-1
	stack_ring3_len equ $-stack_ring3

_ring3_test:
	mov eax,0x78563412
	mov ax,0x6b;0x60为ring0，在ring3时发生权限错误，需要添加ring3数据段
	mov ds,ax
	mov es,ax
	call main
	jmp $

	gdt_ptr dw 0x0
			dd 0x0
