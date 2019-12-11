#include <stdio.h>
#include <stdlib.h>
#include <px4_tasks.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_module.h>
#include <termios.h>
#include <unistd.h>
#include <px4_config.h>
#include <chip/ar_config.h>

static int _uart_fd = -1;

class RFUart: public ModuleBase<RFUart>
{
public:
	RFUart();

	virtual ~RFUart();

	static int task_spawn(int argc, char *argv[]);

	static RFUart *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void run() override;

	int print_status() override;

private:
	static void	cycle_trampoline(void *arg);

	void transceiver();

	void handle_uart_data_send_to_sram();

	void handle_sram_data_send_to_uart();

	int open_uart(int baud, const char *uart_name);
};

RFUart::RFUart() { }

RFUart::~RFUart() { }

int RFUart::print_status()
{
	PX4_INFO("Running as task");

	if (_uart_fd < 0) {
		PX4_INFO("Bad fd number");

	} else {
		PX4_INFO("uart fd = %d", _uart_fd);
	}

	return PX4_OK;
}

int RFUart::custom_command(int argc, char *argv[])
{
	return 0;
}

int RFUart::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 1;
}

void RFUart::transceiver()
{
	_uart_fd = open_uart(115200, "/dev/ttyS7");

	while (!should_exit() && (_uart_fd > 0)) {
		handle_uart_data_send_to_sram();

		handle_sram_data_send_to_uart();

		usleep(5000);
	}
}

void RFUart::handle_uart_data_send_to_sram()
{
	struct pollfd fds[1] = {};
	fds[0].fd = _uart_fd;
	fds[0].events = POLLIN;

	ssize_t nread = 0;
	uint8_t buf[512];

	const int timeout = 10;

	if (poll(&fds[0], 1, timeout) <= 0) {
		return;
	}

	if ((nread = ::read(fds[0].fd, buf, sizeof(buf))) <= 0) {
		return;
	}

	buf[nread] = '\0';

	do {
		char *wr_pos;
		char *rd_pos;
		char *tail;
		char *head;

		STRU_SramBuffer *uartBuffer = (STRU_SramBuffer *)SRAM_UART_TO_SESSION1_DATA_ST_ADDR;

		wr_pos = (char *)uartBuffer->header.buf_wr_pos;
		rd_pos = (char *)uartBuffer->header.buf_rd_pos;
		head = (char *)uartBuffer->buf;
		tail = (char *)SRAM_UART_TO_SESSION1_DATA_END_ADDR;

		for (ssize_t i = 0; i < nread; i++) {
			*wr_pos = buf[i];

			wr_pos++;

			if (wr_pos > tail) {
				wr_pos = head;
			}

			if ((wr_pos == rd_pos) && (wr_pos != head)) {
				PX4_INFO("wr_pos == rd_pos, may miss uart info");
			}
		}

		uartBuffer->header.buf_wr_pos = (uint32_t)wr_pos;
	} while (0);
}

int RFUart::open_uart(int baud, const char *uart_name)
{
	int speed = baud;
	int fd;

	fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		return fd;
	}

	struct termios uart_config;

	int termios_state;

	if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(fd);
		return -1;
	}

	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
		::close(fd);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		::close(fd);
		return -1;
	}

	do {
		struct termios uart_config1;

		tcgetattr(fd, &uart_config1);

		uart_config1.c_cflag |= CRTSCTS;

		tcsetattr(fd, TCSANOW, &uart_config1);

	} while (0);

	return fd;
}

void RFUart::handle_sram_data_send_to_uart()
{
	char *wr_pos;
	char *rd_pos;
	char *tail;
	char *head;

	uint8_t data_buf_proc[512] = {0};
	uint32_t u32_rcvLen = 0;

	STRU_SramBuffer *uartBuffer = (STRU_SramBuffer *)SRAM_SESSION1_TO_UART_DATA_ST_ADDR;

	wr_pos = (char *)uartBuffer->header.buf_wr_pos;
	rd_pos = (char *)uartBuffer->header.buf_rd_pos;
	head = (char *)uartBuffer->buf;
	tail = (char *)SRAM_SESSION1_TO_UART_DATA_END_ADDR;

	while ((wr_pos != rd_pos) && (u32_rcvLen < 512)) {
		data_buf_proc[u32_rcvLen++] = *rd_pos;

		rd_pos++;

		if (rd_pos > tail) {
			rd_pos = head;
		}
	}

	uartBuffer->header.buf_rd_pos = (uint32_t)rd_pos;

	if (u32_rcvLen > 0) {
		write(_uart_fd, data_buf_proc, u32_rcvLen);
	}
}

RFUart *RFUart::instantiate(int argc, char *argv[])
{
	return new RFUart();
}

void RFUart::run()
{
	transceiver();
}

void RFUart::cycle_trampoline(void *arg)
{
	RFUart *dev = reinterpret_cast<RFUart *>(arg);

	if (dev == nullptr) {

		dev = new RFUart();

		if (dev == nullptr) {
			PX4_ERR("alloc failed");
			return;
		}

		_object = dev;
	}

	dev->transceiver();
}

int RFUart::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rfuart", SCHED_DEFAULT, SCHED_PRIORITY_ACTUATOR_OUTPUTS, 1800,
				      (px4_main_t)&run_trampoline, nullptr);

	PX4_INFO("task spawn %d \n", _task_id);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}





extern "C" __EXPORT int rfuart_main(int argc, char *argv[]);

int rfuart_main(int argc, char *argv[])
{
	return RFUart::main(argc, argv);
}

