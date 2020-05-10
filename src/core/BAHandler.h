/*
* Bundle adjustment using ceres
*/

#include <vector>
#include <opencv2/core/core.hpp>
#include <map>
#include "datastructs/Measurement.h"

class BAHandler {
public:
	// void adjustBundle();

	void adjustBundle_ceres_nocopy();

	void adjustBundle_ceres_local_nocopy();

	void adjustBundle_ceres_local_nocopy(std::vector<Measurement::Ptr> &ms);

	// void adjustBundle_ceres_local_fixPoints_nocopy(std::vector<Measurement::Ptr> &ms);

};
