	org 0x7c00
	jmp code_start
code_start:
	mov ax,cs      
	mov ds,ax
	mov es,ax
	mov ss,ax
	mov fs,ax
	mov sp,0x7c00
	push 0x0
	push 0x2;开始读取的扇区编号
	push 0x0
	push 0x900;读取的数据放置于内存的开始地址
	push 55;读取扇区的个数
	call read_disk
	push 0x0
	push 0x20
	push 0x0
	push 0x7e00
	push 35
	call read_disk
	jmp 0x900
;int* read_disk(short section_num,short* start_addr,short start_section)
;section_num读取扇区的个数,start_addr读取的数据放置于内存的开始地址，start_section开始读取的扇区编号
;read_disk(10,0x900,2)
read_disk:
	mov bx,sp
	;设置读取扇区的个数
	mov ax,[bx+2];call指令会将ip压栈
	mov dx,0x1f2
	out dx,al
	;设置读扇区的起始地址，扇区编号从0开始
	mov eax,[bx+0x8]
	mov dx,0x1f3
	out dx,al
	shr eax,8
	mov dx,0x1f4
	out dx,al
	shr eax,8
	mov dx,0x1f5
	out dx,al
	shr eax,8
	and al,0xf
	or al,0xe0
	mov dx,0x1f6
	out dx,al
	;开始读操作
	mov dx,0x1f7
	mov al,0x20                        
	out dx,al
not_ready:
	nop
	in al,dx
	and al,0x88
	cmp al,0x08
	jnz not_ready
	mov ax,[bx+2]
	mov dx, 256
	mul edx;mul汇编指令，默认乘以ax，结果放在ax中
	xor ecx,ecx
	mov ecx, eax
	mov dx, 0x1f0
	mov ebx,[bx+4]
go_on_read:
	in ax,dx
	mov [ebx],ax;读取的内容从bx处开始存放
	add ebx,2
	cmp ebx,0x10000
	jz .equal
	jmp .not_equal
.equal:
	mov ebx,0x0
	push ax
	mov ax,ds
	add ax,0x1000
	mov ds,ax
	pop ax
.not_equal:
	loop go_on_read;loop指令，cx存放循环次数
	ret
	times 510-($-$$) db 0
	db 0x55,0xaa
