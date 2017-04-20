/*
 * Grabber.h
 *
 *  Created on: Apr 7, 2017
 *      Author: abds
 */

#ifndef GRABBER_H_
#define GRABBER_H_

#include "SaveOption.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

class SimpleOpenNIViewer
{
	public:
		SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

		void cloud_cb_ (const PointCloud::ConstPtr &cloud);

		void run ();

		pcl::visualization::CloudViewer viewer;
};


#endif /* GRABBER_H_ */
