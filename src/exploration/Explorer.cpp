#include "exploration/Explorer.hpp"
#include "porting.hpp"

namespace exploration {

Explorer::Explorer(porting::DroneLayer *porting) : my_id{}, porting_{porting} {
	init_median_filter_f(&medFilt, 5);
	init_median_filter_f(&medFilt_2, 5);
	init_median_filter_f(&medFilt_3, 13);
}

void Explorer::init() {
	auto address = porting_->config_block_radio_address();
	my_id = static_cast<uint8_t>(address & 0x00000000FFU);
}

} // namespace exploration
