	global add
add:
	mov eax,[esp+4]
	mov edx,[esp+8]
	add eax,edx
	ret
TODO:
	(1)将boot.S中跳转到保护模式之后的代码逻辑放到start.S中
	(2)boot.S中只保留实模式和跳转保护模式的代码
	(3)jmp protected_mode_selector:0x202000
