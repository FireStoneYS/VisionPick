/*
 * Init.cpp
 *
 *  Created on: Apr 7, 2017
 *      Author: abds
 */


#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include "Init.h"

using namespace std;

const XnMapOutputMode NiOption::get_default_output_mode(){
    XnMapOutputMode outputMode = {XN_VGA_X_RES, XN_VGA_Y_RES, 30};
    return outputMode;
}

void NiOption::set_output_mode(const XnMapOutputMode &outputMode){
	map_mode.nFPS  = outputMode.nFPS;
	map_mode.nXRes = outputMode.nXRes;
	map_mode.nYRes = outputMode.nYRes;
}

void NiOption::SingleInit(){
	result_val = context.Init();
	printError();
	result_val = depth_generator.Create(context);
	printError();
	result_val = image_generator.Create(context);
	printError();
	result_val = depth_generator.SetMapOutputMode(get_default_output_mode());
	printError();
	result_val = image_generator.SetMapOutputMode(get_default_output_mode());
	printError();
	depth_generator.GetAlternativeViewPointCap().SetViewPoint( image_generator );
	printError();
	result_val = context.StartGeneratingAll();
	printError();
	result_val = context.WaitNoneUpdateAll();
	printError();
}

void NiOption::DoubleInit(){
	result_val = context.Init();
	printError();
	result_val = depth_generator.Create(context);
	printError();
	result_val = image_generator.Create(context);
	printError();
	result_val = context.EnumerateProductionTrees(XN_NODE_TYPE_IMAGE,NULL,image_node_info_list);
	printError();
	result_val = context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH,NULL,depth_node_info_list);
	printError();

	for(xn::NodeInfoList::Iterator nodeIt = image_node_info_list.Begin();nodeIt != image_node_info_list.End();nodeIt++)
	{
		xn::NodeInfo imageinfo = *nodeIt;
		result_val = context.CreateProductionTree(imageinfo);
		printError();
		imageinfo.GetInstance(image_generator);
		result_val = image_generator.SetMapOutputMode(get_default_output_mode());
		printError();
		nImageGen.push_back(image_generator);
	}
	cout<<"image push back success"<<endl;
	for(xn::NodeInfoList::Iterator nodeIt = depth_node_info_list.Begin();nodeIt != depth_node_info_list.End();nodeIt++)
	{
		xn::NodeInfo depthinfo = *nodeIt;
		result_val = context.CreateProductionTree(depthinfo);
		printError();
		depthinfo.GetInstance(depth_generator);
		result_val = depth_generator.SetMapOutputMode(get_default_output_mode());
		printError();
		nDepthGen.push_back(depth_generator);
	}
	cout<<"depth push back success"<<endl;
	result_val = nDepthGen[0].GetAlternativeViewPointCap().SetViewPoint(nImageGen[1]);
//	printError();检测返回值出错，但如果更换视点顺序，就没有错误，但是图像不对
	cout<<"image number"<<nDepthGen.size()<<endl;
	result_val = nDepthGen[1].GetAlternativeViewPointCap().SetViewPoint(nImageGen[0]);
//	printError();检测返回值出错，但如果更换视点顺序，就没有错误，但是图像不对
	cout<<"depth number"<<nDepthGen.size()<<endl;
	result_val = context.StartGeneratingAll();
	printError();
	result_val = context.WaitNoneUpdateAll();
	printError();
	cout<<"Init success"<<endl;
}

void NiOption::stop()
{
    context.StopGeneratingAll();
    context.Shutdown();
    cv::destroyAllWindows();
}

void NiOption::printError()
{
    if (result_val != XN_STATUS_OK)
    {
        printf("Error: %s", xnGetStatusString(result_val));
        exit(-1);
    }
}

