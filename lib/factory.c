#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/bug.h>
#include <linux/errno.h>

int isfactory(int *Flag){
	(*Flag) = 1;
	return 0;
}

EXPORT_SYMBOL(isfactory);
