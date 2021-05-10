CC	= cc
PROGRAMS = sample

PREFIXDIR = /usr
LIBDIR = $(PREFIXDIR)/lib
INCDIR = $(PREFIXDIR)/include
LIBS  = sii
AIO_LIBS = rt

LDFLAGS	= $(PROGRAMS).c -l$(LIBS) -l$(AIO_LIBS) -o$(PROGRAMS)

CFLAGS = -g 

RM	= rm -f

all:: 
	$(CC) $(CFLAGS) $(LDFLAGS)


clean::
	$(RM) $(PROGRAMS) core tags MAKEOUT 
	$(RM) *.c.* *.o

oraclean::
	$(RM) *.c.*

