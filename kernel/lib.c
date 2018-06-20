extern void printToScreen(void *src,unsigned int n);
void print(void *src,unsigned int n){
	printToScreen(src,n);
}
void printInt(unsigned int x,int radix){
	char s[20];
	int i,j,n=0,mod;	
	while(x){
		mod=x%radix;
		x=x/radix;
		switch(mod){
			case 0:case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:
				s[n]='0'+mod;
				break;
			case 10:case 11:case 12:case 13:case 14:case 15:
				s[n]='A'+mod-10;
				break;
		}
		n++;
	}
	for(i=0,j=n-1;i<=j;i++,j--){
		char t=s[i];
		s[i]=s[j];
		s[j]=t;
	}
	s[n]='\0';
	print(s,n+1);
}
