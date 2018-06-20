;void printToScreen(void *src,unsigned int n)
;push n
;push src
	extern put_char
	global printToScreen
printToScreen:
	xor ecx,ecx
	mov cx,[esp+8]
	mov ebx,[esp+4]
.not_finish:
	mov al,[ebx]
	inc ebx
	and al,0xff
	push ax
	call put_char
	pop ax
	loop .not_finish
	ret
