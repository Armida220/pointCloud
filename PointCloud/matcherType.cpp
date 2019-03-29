#include "matcherType.h"

namespace feature
{

	std::string matcherTypeToString(MatcherType matcherType)
	{
		switch (matcherType)
		{
		case MatcherType::Knn:
				return "Knn";
		case MatcherType::BF:
				return "BF";
		}
		throw std::out_of_range("invalid matcherType enum");
	}

	MatcherType stringToMatcherType(const std::string& matcherType)
	{
		if (matcherType == "Knn")
			return MatcherType::Knn;
		if (matcherType == "BF")
			return MatcherType::BF;

		throw std::out_of_range("invalid matcherTYpe: " + matcherType);
	}
}