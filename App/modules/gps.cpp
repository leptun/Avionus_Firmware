#include "gps.hpp"
#include <inttypes.h>
#include <config.hpp>
#include <pins.hpp>


namespace modules {
namespace gps {

class UbxGNSS {
	typedef uint8_t U1;
	typedef uint16_t U2;
	typedef uint32_t U4;
	typedef int8_t I1;
	typedef int16_t I2;
	typedef int32_t I4;
	typedef uint8_t X1;
	typedef uint16_t X2;
	typedef uint32_t X4;
	typedef float R4;
	typedef double R8;
	typedef char CH;

	static constexpr uint8_t preamble_sync[2] = { 0xb5, 0x62 };
	enum class Messages : uint16_t {
		// class << 8 | ID
		UBX_ACK_ACK = 0x0105,
		UBX_ACK_NAK = 0x0005,
	};

	class Checksum {
		union {
			struct {
				uint8_t ck_a;
				uint8_t ck_b;
			};
			U2 ck;
		};
	public:
		void reset() {
			ck_a = 0;
			ck_b = 0;
		}
		void process(uint8_t *msg, size_t len) {
			while (len-- > 0) {
				ck_a += *(msg++);
				ck_b += ck_a;
			}
		}
		bool verify(U2 chk) {
			return ck == chk;
		}
	} chk;
};

static UbxGNSS gnss;

void Setup() {
	config::gps_usart->Setup();
	pins::GPS::NRESET.Write(true);
}

}
}
