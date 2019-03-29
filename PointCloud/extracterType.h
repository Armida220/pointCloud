#pragma once

#include <string>

namespace feature
{
	enum ExtracterType
	{
		SIFT = 1,
		SURF
	};


	/**
	 * @brief this function convert feature::ExtracterType to string
	 */
	std::string extracterTypeToString(ExtracterType extracterType);

	/**
	 * @brief this function convert feature::ExtracterType to string
	 */
	ExtracterType stringToExtracterType(const std::string& extracterType);
}