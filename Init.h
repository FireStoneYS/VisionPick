/*
 * Init.h
 *
 *  Created on: Apr 7, 2017
 *      Author: abds
 */

#ifndef INIT_H_
#define INIT_H_

#include <XnCppWrapper.h>
#include <vector>

#define ISDOUBLEKINECT 0
#define NUMBER_OF_KINECT 2


class NiOption
{
	private:
		xn::Context            	&context;
		xn::DepthGenerator     	&depth_generator;
		xn::ImageGenerator     	&image_generator;

		XnMapOutputMode    map_mode;
	public:
		std::vector<xn::ImageGenerator> nImageGen;
		std::vector<xn::DepthGenerator> nDepthGen;
		xn::DepthMetaData depthMD;
		xn::ImageMetaData imageMD;
		XnStatus           result_val;
		xn::NodeInfoList 		image_node_info_list;
		xn::NodeInfoList 		depth_node_info_list;
		NiOption(xn::Context &conText, xn::ImageGenerator &imageGenerator,xn::DepthGenerator &depthGenerator)
		                : context(conText),image_generator(imageGenerator), depth_generator(depthGenerator),
		                  result_val(XN_STATUS_OK){};

		~NiOption() { stop(); }
		inline const XnMapOutputMode get_default_output_mode();
		void set_output_mode(const XnMapOutputMode &outputMode);
	    void SingleInit();
	    void DoubleInit();
		void stop();
		void printError();
};



#endif /* INIT_H_ */
