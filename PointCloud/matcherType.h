#pragma once

#include <string>

namespace feature
{
	enum MatcherType
	{
		Knn = 1,
		BF
	};

	/**
	 * @brief this function convert feature::marcherType to string
	 */
	std::string matcherTypeToString(MatcherType matcherType);

	/**
	 * @brief this function convert feature::marcherType to string
	 */
	MatcherType stringToMatcherType(const std::string& matcherType);

}