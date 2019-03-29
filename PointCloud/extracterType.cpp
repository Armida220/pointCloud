#include "extracterType.h"

namespace feature
{

	std::string extracterTypeToString(ExtracterType extracterType)
	{
		switch (extracterType)
		{
		case ExtracterType::SIFT:
			return "SIFT";
		case ExtracterType::SURF:
			return "SURF";
		}
		throw std::out_of_range("invalid extracterType enum");

	}

	ExtracterType stringToExtracterType(const std::string& extracterType)
	{
		if (extracterType == "SIFT")
			return ExtracterType::SIFT;
		if (extracterType == "SURF")
			return ExtracterType::SURF;

		throw std::out_of_range("invalid extracterType: " + extracterType);
	}
}
