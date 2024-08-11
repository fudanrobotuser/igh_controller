CC = gcc
CFLAGS = -I. -Wall -I/usr/local/etherlab/include
LDFLAGS = -L/usr/local/etherlab/lib -lpthread -lrt -lethercat

# TARGET = main
# SRCS = main.c ecat_data_buffer.c

# TARGET = csp_torque
# SRCS = csp_torque.c

# TARGET = csp_all
# SRCS = csp_all.c

TARGET = T
# SRCS = main_0710.c ecat_data_buffer.c
SRCS = main_0803.c ecat_data_buffer.c

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)


TARGET2 = CSP
# SRCS = main_0710.c ecat_data_buffer.c
SRCS2 = main_0710.c ecat_data_buffer.c

$(TARGET2): $(SRCS2)
	$(CC) $(CFLAGS) -o $(TARGET2) $(SRCS2) $(LDFLAGS)

TARGET3 = CSP_T
# SRCS = main_0710.c ecat_data_buffer.c
SRCS3 = main_0809_csp_t.c ecat_data_buffer.c

$(TARGET3): $(SRCS3)
	$(CC) $(CFLAGS) -o $(TARGET3) $(SRCS3) $(LDFLAGS)


clean:
	rm -f $(TARGET) *.o $(TARGET2) *.o $(TARGET3) *.o

