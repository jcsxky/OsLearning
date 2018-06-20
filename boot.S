	org 0x900
	jmp code_start
	;gdt数组
	gdt_dummy dd 0x0
						dd 0x0
	gdt_code_ring0 dd 0x00000000
					dd  0x00c49800;运行时修改为段界限40000，段基址0xc000000
	gdt_video_ring0 dd 0x00000000
		      dd 0x00c49200
	gdt_tss dd 0x00000000
					dd 0xc0808900;运行时修改为段界限40000,等待运行时加载，段基址0xc000000
	gdt_code_ring3 dd 0x0000ffff
					dd  0x00cff800
	gdt_stack_ring3 dd 0x0
		      dd 0x00c0f200
	gdt_stack_ring0 dd 0x0
					dd 0x00c09200
	;调用门gdt，目标gdt_code
	gdt_call_gate dd 0x00400000
					dd 0x0000ec00
	gdt_code_ring0_test_gate dd 0x0000ffff
					dd 0xc0cf9800
	gdt_stack_ring0_test_gate dd 0x0000ffff
					dd 0x00c09200
	gdt_call_far dd 0x00080000
					dd 0x00008c00
	gdt_code_test_interrupt dd 0x0
					dd  0xc0c49800
	gdt_data_ring0 dd 0x00000000
					dd 0x00c49200
	gdt_data_ring3 dd 0x00000000
					dd 0x00c4f200
	;gdt长度和界限
	gdt_size equ $-gdt_dummy
	gdt_limit equ gdt_size-1
	;选择子
	code_ring0_selector equ 0x0008
	video_ring0_selector equ 0x0010
	tss_selector equ 0x0018
	code_ring3_selector equ 0x0020+3
	stack_ring3_selector equ 0x0028+3
	stack_ring0_selector equ 0x0030
	gdt_call_gate_selector equ 0x0038+3
	gdt_code_ring0_test_gate_selector equ 0x40
	gdt_stack_ring0_test_gate_selector equ 0x48
	gdt_call_far_selector equ 0x50
	gdt_code_test_interrupt_selector equ 0x58+3
	data_ring0_selector equ 0x60
	data_ring3_selector equ 0x68+3
	;加载gdt寄存器
	gdt_ptr dw gdt_limit
			dd gdt_dummy


[bits 16]
code_start:
	mov ax,cs
	mov ds,ax
	mov es,ax
	mov ss,ax
	mov fs,ax
	mov gs,ax

	in al,0x92
	or al,0000_0010B
	out 0x92,al
	xor eax,eax
	mov ax,ds
	shl eax,4
	add eax,gdt_dummy
	mov [gdt_ptr+2],eax
	lgdt [gdt_ptr]
	mov eax, cr0
	or eax, 0x00000001
	mov cr0, eax
	jmp dword code_ring0_selector:protection_mode_start;跳入保护模式

[bits 32]
tss_segment:
		dd 0
		dd stack_ring0_test_gate_len	;ring0 stack segment base
		dd gdt_stack_ring0_test_gate_selector ;ring0 stack pointer
		dd 0	;ring1 stack
		dd 0
		dd 0	;ring2 stack
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0
		dd 0x00000010
		dd 0
		dw 0
		dw $-tss_segment+2
		db 0xff
	tss_len equ $-tss_segment

ring3_code:
	mov eax,0x12345678
	;push 0x12345678
	int 0x80
	;即将进入ring0
	;call gdt_call_gate_selector:0
	jmp $

code_ring0_test_gate:
	mov eax,0x78563412
	mov ax,video_ring0_selector
	mov gs,ax
	mov byte [gs:0x0],'2'
	mov byte [gs:0x01],0xa5
	;push 0x78563412
	;中断返回用iretd，调用门返回用retf
	;中断测试，retf，测试完成时恢复
	;出现错误时错误码被压栈，需要先弹出错误码，pop eax
	iretd

code_test_interrupt:
	;push 0x78563412
	mov ax,video_ring0_selector
	mov gs,ax
	inc byte [gs:0x0]
	;mov byte [gs:0x0],'3'
	;mov byte [gs:0x01],0xa5
	;inc	byte [gs:((80 * 0 + 70) * 2)]	; 屏幕第 0 行, 第 70 列。
	mov	al, 20h
	out	20h, al				; 发送 EOI
	out 0A0h, al
	iretd
	;jmp $


ALIGN	32
[bits	32]
stack_ring3:
	times 200 db 0
	top_of_stack_ring3	equ	$-stack_ring3-1
	stack_ring3_len equ $-stack_ring3

ALIGN	32
[bits	32]
stack_ring0_test_gate:
	times 200 db 0
	top_of_stack_ring0_test_gate	equ	$-stack_ring0_test_gate-1
	stack_ring0_test_gate_len equ $-stack_ring0_test_gate

ALIGN	32
[bits	32]
stack_ring0:
	times 0x1000 db 0
	top_of_stack_ring0	equ	$-stack_ring0-1
	stack_ring0_len equ $-stack_ring0

protection_mode_start:
	mov ax,0x60
	mov ds,ax
	mov es,ax
	;设置TSS段的基址和段界限tss_segment
	xor ax,ax
	mov ax,tss_len-1
	or [gdt_tss],ax
	xor ax,ax
	mov eax,tss_segment
	or [gdt_tss+2],ax
	shr eax,16
	or [gdt_tss+7],ah
	or [gdt_tss+4],al
	mov ax,0x0018
	ltr ax
	;设置stack_ring0段的基址和段界限
	xor ax,ax
	mov ax,stack_ring0_len-1
	mov [gdt_stack_ring0],ax
	xor ax,ax
	mov eax,stack_ring0
	mov [gdt_stack_ring0+2],ax
	shr eax,16
	mov [gdt_stack_ring0+7],ah
	mov [gdt_stack_ring0+4],al
	;测试ring0堆栈段
	mov ax,stack_ring0_selector
	mov ss,ax
	mov eax,top_of_stack_ring0
	mov esp,eax
	;push 0x12345678
	;设置stack_ring3段的基址和段界限
	xor ax,ax
	mov ax,stack_ring3_len-1
	mov [gdt_stack_ring3],ax
	xor ax,ax
	mov eax,stack_ring3
	mov [gdt_stack_ring3+2],ax
	shr eax,16
	mov [gdt_stack_ring3+7],ah
	mov [gdt_stack_ring3+4],al
	;设置显存段的基址和段界限
	xor ax,ax
	mov ax,0x7fff
	or [gdt_video_ring0],ax
	xor ax,ax
	mov eax,0xb8000
	or [gdt_video_ring0+2],ax
	shr eax,16
	or [gdt_video_ring0+7],ah
	or [gdt_video_ring0+4],al
	mov ax,video_ring0_selector
	mov gs,ax
	mov byte [gs:0x0],'1'
	mov byte [gs:0x01],0xa5
	;设置stack_ring0_test_gate段的基址和段界限
	xor ax,ax
	mov ax,stack_ring0_len-1
	or [stack_ring0_test_gate],ax
	xor ax,ax
	mov eax,stack_ring3
	or [stack_ring0_test_gate+2],ax
	shr eax,16
	or [stack_ring0_test_gate+7],ah
	or [stack_ring0_test_gate+4],al
from_ring3_to_ring0_test:
	mov eax,0x12345678
	;准备跳入ring3,设置tss中ring0段的ss和esp
	;mov eax,esp
	;mov [tss_segment+4],eax
	;mov ax,ss
	;mov [tss_segment+8],ax
	;mov eax,from_ring3_to_ring0_test
	;mov [gdt_call_gate+2],ax
	;shr eax,16
	;mov [gdt_code+6],ax
	;设置call gate中选择子段的base addr
	;中断测试时注释掉，测试完成后打开
	;mov eax,code_ring0_test_gate
	;mov [gdt_code_ring0_test_gate+2],ax
	;shr eax,16
	;mov [gdt_code_ring0_test_gate+4],al
	;mov [gdt_code_ring0_test_gate+7],ah
	;测试中断描述符
	xor eax,eax
	mov bx,255
	mov cx,bx
	;mov eax,code_ring0_test_gate
	
idt_init:
	mov bx,255
	sub bx,cx
	mov ax,8
	mul bx
	mov bx,ax
	mov eax,code_ring0_test_gate
	mov [idt_label+bx],ax
	shr eax,16
	mov [idt_label+bx+6],ax
	loop idt_init
	mov eax,idt_label
	mov [idt_ptr+2],eax
	mov ax,idt_len-1
	mov [idt_ptr],ax
	cli
	lidt [idt_ptr]
	mov eax,init_8259a
	mov [gdt_call_far],ax
	shr eax,16
	mov [gdt_call_far+6],ax
	call gdt_call_far_selector:0

	;mov eax,code_test_interrupt
	;mov [system_call],ax
	;shr eax,16
	;mov [system_call+6],ax

	;测试时钟中断
	;mov eax,code_test_interrupt
	;mov [idt_label+256],ax
	;shr eax,16
	;mov [idt_label+256+6],ax
	;测试分页
	;在分页之前代码运行在物理地址0x900-0x7bff之内，但是分页之后cpu需要根据cr3寄存器的值进行地址转换，gdt的基址
	;要设置为0xc0000000，这样就导致没有一个合适的时机来设置这个值。设置在分页之前代码访问的段基址就不对，设置在
	;分页之后cpu按线性地址访问，同样不对。解决的办法是将0-1M之内的内存和3G之上的内存都映射到0-1G
	mov eax,0x100000
	mov cr3,eax
	mov eax,0x100007;+0x1000
	mov ebx,0x0;+4
	mov ecx,0x100
init_pde:
	add eax,0x1000
	mov [ebx+0x100000],eax
	mov [ebx+0xc00+0x100000],eax
	add ebx,0x4
	loop init_pde
	mov eax,0x7
	mov ebx,0x1000
	mov ecx,0x40000
init_pte:
	mov [ebx+0x100000],eax
	add eax,0x1000
	add ebx,0x4
	loop init_pte
	mov eax, cr0
	or eax, 0x80000000
	mov cr0,eax
	;进入分页之后，cpu访问的都是虚拟地址，此时os使用的虚拟地址在0xc0000000之上
	;段的基址都要调整到0xc0000000之上
	;add esp,0xc0000000
	mov byte [gdt_code_ring0+7],0xc0
	mov byte [gdt_ptr+5],0xc0
	mov byte [gdt_data_ring0+7],0xc0
	;设置gdt_stack_ring0段的段基址
	mov byte [gdt_stack_ring0+7],0xc0
	mov word [gdt_stack_ring0+2],0x0
	mov byte [gdt_stack_ring0+4],0x0
	lgdt [gdt_ptr]
	;准备页表页目录，跳入ring3
	mov eax,0x78563412
	push stack_ring3_selector
	push top_of_stack_ring3
	push code_ring3_selector
	push ring3_code
	;开中断指令，必须等到页表页目录建立好之后才能开中断，否则中断会影响页表页目录的建立，出现各种莫名其妙的
	;的错误(可能是由于中断处理程序没有保存现场)，导致页表建立不成功，影响后续指令的执行，2018-03-17
	;进入中断处理程序之前一定要保存现场，否则会影响其他程序的执行
	;sti
	call kernel_init
	jmp dword code_ring0_selector:0x00202000
	retf
	;此时cpu处于ring3
	;jmp $

;加载内核到0x7e00处
kernel_init:
	mov dx, [0x7e00+42]
	mov ebx,[0x7e00+28]
	add ebx,0x7e00
	mov cx, [0x7e00+44]
each_segment:
	cmp byte [ebx],0
	je pt_null
	push dword [ebx+16]
	mov eax,[ebx+4]	
	add eax,0x7e00
	push eax
	push dword [ebx+8]
	call mem_cpy
	add esp,12
pt_null:
	add ebx, edx
	loop each_segment
	ret
mem_cpy:		      
   cld
   push ebp
   mov ebp, esp
   push ecx
   mov edi,[ebp+8]	   ; dst
   mov esi,[ebp+12]	   ; src
   mov ecx,[ebp+16]	   ; size
   rep movsb
   pop ecx		
   pop ebp
   ret

;以下为idr
ALIGN	32
[BITS	32]
idt_label:
	%rep 11
		dd 0x00080000
		dd 0x0000ee00
	%endrep
	net_interrupt1:
		dd 0x00580000
		dd 0x0000ee00
	;%rep 86
	;	dd 0x00080000
	;	dd 0x0000ee00
	;%endrep
	;system_call:
	;	dd 0x00400000
	;	dd 0x0000ee00
	%rep 244
		dd 0x00080000
		dd 0x0000ee00
	%endrep
	idt_len equ $-idt_label
	idt_ptr dw idt_len-1
			dd idt_label

init_8259a:
	mov	al, 011h
	out	020h, al	; 主8259, ICW1.
	out	0A0h, al	; 从8259, ICW1.
	mov	al, 020h	; IRQ0 对应中断向量 0x20
	out	021h, al	; 主8259, ICW2.
	mov	al, 028h	; IRQ8 对应中断向量 0x28
	out	0A1h, al	; 从8259, ICW2.
	mov	al, 004h	; IR2 对应从8259
	out	021h, al	; 主8259, ICW3.
	mov	al, 002h	; 对应主8259的 IR2
	out	0A1h, al	; 从8259, ICW3.
	mov	al, 001h
	out	021h, al	; 主8259, ICW4.
	out	0A1h, al	; 从8259, ICW4.
	mov	al, 11111011b	; 仅仅开启定时器中断
	;mov	al, 11111111b	; 屏蔽主8259所有中断
	out	021h, al	; 主8259, OCW1.
	mov	al, 11111101b	; 屏蔽从8259所有中断
	out	0A1h, al	; 从8259, OCW1.
	retf
