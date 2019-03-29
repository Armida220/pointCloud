#include "image.h"

namespace imageIO
{

	void getFiles(const std::string& path, std::vector<std::string>& fileList, const std::string& fileType)
	{
		intptr_t    hFile = 0;
		struct _finddata_t fileinfo;
		std::string p;

		if ((hFile = _findfirst(p.assign(path).append("\\*." + fileType).c_str(), &fileinfo)) != -1) {
			do
			{
				if ((fileinfo.attrib &  _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					{
						fileList.push_back(p.assign(path).append("\\").append(fileinfo.name));
						getFiles(p.assign(path).append("\\").append(fileinfo.name), fileList);
					}
				}
				else
				{
					fileList.push_back(p.assign(path).append("\\").append(fileinfo.name));
				}

			} while (_findnext(hFile, &fileinfo) == 0);

			_findclose(hFile);
		}
	}



	void readImage(const std::string& path, std::vector<cv::Mat>& imageVec)
	{
		std::vector<std::string> imageList;
		getFiles(path, imageList, "jpg");

		cv::Mat image;
		for (std::string imageName : imageList)
		{
			std::cout << "find image '" << imageName << "'"<< std::endl;
			image = cv::imread(imageName);
			if (image.empty())
			{
				std::cout << "can't read from " << imageName << std::endl;
				continue;
			}
			imageVec.push_back(image);
		}
	}




}