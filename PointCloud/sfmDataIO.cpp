#include "sfmDataIO.h"


namespace sfm
{
	void save2Yml(
		const std::string& file_name,
		const std::vector<cv::Mat>& rotations,
		const std::vector<cv::Mat>& motions,
		//const std::vector<cv::Point3f>& structure,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::Vec3b>& colors
	)
	{
		int n = (int)rotations.size();

		cv::FileStorage fs(file_name,cv:: FileStorage::WRITE);
		fs << "Camera Count" << n;
		fs << "Point Count" << (int)structure.size();

		fs << "Rotations" << "[";
		for (size_t i = 0; i < n; ++i)
		{
			fs << rotations[i];
		}
		fs << "]";

		fs << "Motions" << "[";
		for (size_t i = 0; i < n; i++)
		{
			fs << motions[i];
		}
		fs << "]";

		fs << "Points" << "[";
		for (size_t i = 0; i < structure.size(); ++i)
		{
			fs << structure[i];
		}
		fs << "]";


		fs << "Colors" << "[";
		for (size_t i = 0; i < colors.size(); ++i)
		{
			fs << colors[i];
		}
		fs << "]";

		fs.release();
	}


	bool save2Ply(
		const std::string& filename,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::Vec3b>& colors
	)
	{

		//Create the stream and check it is ok
		std::ofstream stream(filename.c_str());
		if (!stream.is_open())
			return false;


		bool bOk = false;

		// Count how many views having valid poses:
		//int view_with_pose_count = 0;
		//if (b_extrinsics)
		//{
		//	for (const auto& view : sfmData.getViews())
		//	{
		//		view_with_pose_count += sfmData.isPoseAndIntrinsicDefined(view.second.get());
		//	}
		//}

		stream << "ply"
			<< '\n' << "format ascii 1.0"
			<< '\n' << "element vertex "
			//// Vertex count: (#landmark + #view_with_valid_pose)
			//<< ((b_structure ? sfmData.getLandmarks().size() : 0) +
			//	view_with_pose_count)
			<< structure.size()
			<< '\n' << "property double x"
			<< '\n' << "property double y"
			<< '\n' << "property double z"
			<< '\n' << "property uint8 red"
			<< '\n' << "property uint8 green"
			<< '\n' << "property uint8 blue"
			<< '\n' << "end_header" << std::endl;

		//if (b_extrinsics)
		//{
		//	for (const auto& view : sfmData.getViews())
		//	{
		//		if (sfmData.isPoseAndIntrinsicDefined(view.second.get()))
		//		{
		//			const geometry::Pose3 pose = sfmData.getPose(*(view.second.get())).getTransform();
		//			stream << pose.center().transpose()
		//				<< " 0 255 0" << "\n";
		//		}
		//	}
		//}

		//if (b_structure)
		//{
		//	const sfmData::Landmarks& landmarks = sfmData.getLandmarks();
		//	for (sfmData::Landmarks::const_iterator iterLandmarks = landmarks.begin();
		//		iterLandmarks != landmarks.end();
		//		++iterLandmarks) {
		//		stream << iterLandmarks->second.X.transpose() << " "
		//			<< (int)iterLandmarks->second.rgb.r() << " "
		//			<< (int)iterLandmarks->second.rgb.g() << " "
		//			<< (int)iterLandmarks->second.rgb.b() << "\n";
		//	}
		//}

		for (int i=0;i<structure.size();++i)
		{
			cv::Point3d pt = structure[i];
			cv::Vec3b color = colors[i];
			stream << pt.x << " " << pt.y << " " << pt.z << " "
				   << (int)color[0] << " " << (int)color[1] << " " << (int)color[2]
				   << "\n";
		}


		stream.flush();
		bOk = stream.good();
		stream.close();

		return bOk;

	}

}