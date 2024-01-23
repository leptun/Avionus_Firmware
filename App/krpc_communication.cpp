#include <krpc_cnano/communication.h>
#include <krpc_cnano/error.h>

#include <modules/usb.hpp>
#include <FreeRTOS.h>
#include <stream_buffer.h>

krpc_error_t krpc_open(krpc_connection_t *connection,
		const krpc_connection_config_t *arg) {
	return KRPC_OK;
}

krpc_error_t krpc_close(krpc_connection_t connection) {
	return KRPC_OK;
}

krpc_error_t krpc_read(krpc_connection_t connection, uint8_t *buf,
		size_t count) {
	size_t read = 0;
	while (true) {
		read += xStreamBufferReceive(modules::usb::krpc_rx_stream, buf + read, count - read, portMAX_DELAY);
		if (read == count)
			return KRPC_OK;
	}
}

krpc_error_t krpc_write(krpc_connection_t connection, const uint8_t *buf,
		size_t count) {
	size_t written = 0;
	while (true) {
		written += xStreamBufferSend(modules::usb::krpc_tx_stream, buf + written, count - written, portMAX_DELAY);
		if (written == count)
			return KRPC_OK;
	}
}

