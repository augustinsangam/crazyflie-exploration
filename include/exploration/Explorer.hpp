#ifndef EXPLORER_HPP
#define EXPLORER_HPP

#include "median_filter.hpp"
#include "porting.hpp"

namespace exploration {

class Explorer {
	std::uint8_t my_id; // TODO(): RENAME
	struct MedianFilterFloat medFilt, medFilt_2, medFilt_3;
	porting::DroneLayer *porting_;

public:
	explicit Explorer(porting::DroneLayer *porting);

	void init();
};

} // namespace exploration

#endif /* EXPLORER_HPP */
