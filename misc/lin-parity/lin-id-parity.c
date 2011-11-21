#include <stdio.h>

int main(void)
{
	unsigned int x, p0, p1;
	for (x = 0; x <= 0x3f; x++) {
		p0 = (x ^ (x >> 1) ^ (x >> 2) ^ (x >> 4)) & 0x1;
		p1 = ~(((x >> 1) ^ (x >> 3) ^ (x >> 4) ^ (x >> 5))) & 0x1;
		printf("%s0x%02x%s", x & 0x7? "": "\n\t",
			((p1 & 1) << 7) | ((p0 & 0x1) << 6),
			x!=0x3f? ",": "\n");
	}
}
