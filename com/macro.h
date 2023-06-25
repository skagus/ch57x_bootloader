#pragma once

#ifdef FALSE
	#undef FALSE
	#undef TRUE
#endif
#define UNUSED(x)					(void)(x)

#define NOT(x)						(!(x))
#define BRK_IF(cond)				do{ if (cond) while(1); }while(0)
#define ASSERT(cond)				BRK_IF(NOT(cond))

#define IF_THEN(cond, check)		ASSERT(NOT(exp) || (check))


#define STATIC_ASSERT(exp, str)		static_assert(exp, str);


#define BIT(shift)					(1 <<(shift))
#define BIT_SET(dst, mask)			((dst) |= (mask))
#define BIT_CLR(dst, mask)			((dst) &= ~(mask))
#define BIT_TST(dst, mask)			((dst) & (mask))

#define MEMSET_SIZE(obj, val)		memset(obj, val, sizeof(obj))

#define ALIGN_DN(nNum, nAlign)		(((nNum) / (nAlign)) * (nAlign))
#define ALIGN_UP(nNum, nAlign)		((((nNum) + (nAlign) - 1) / (nAlign)) * (nAlign))

#define MAX(a, b) 					((a) > (b) ? (a) : (b))
#define MIN(a, b) 					((a) < (b) ? (a) : (b))
