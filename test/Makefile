CXXFLAGS =	-O0 -g -Wall -fmessage-length=0

OBJS =		test.o

LIBS =		

TARGET =	test

$(TARGET):	$(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(LIBS) 

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
