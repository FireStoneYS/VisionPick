///*
// * Recognition.cpp
// *
// *  Created on: Apr 9, 2017
// *      Author: abds
// */
//
//#include "Registration.h"
//#include "Feature.h"
//#include "Segmentation.h"
//#include "Filter.h"
//#include "Normal.h"
//#include "Recognition.h"
//#include "Serial.h"
//#include "SVM.h"
//#include "socketmove.h"
//
//
//#include <pcl/registration/transforms.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/common/centroid.h>
//#include <pcl/segmentation/extract_clusters.h>
//using namespace std;
//
//pcl::visualization::PCLVisualizer view("Point Cloud");
//PointCloud::Ptr scene(new PointCloud);
//PointCloud::Ptr scene_filter(new PointCloud);
//
//GlobalPara globalpara;
//void *son_thread(void * data){
//	time_t start,stop;
//
//	start = time(NULL);
//	ParameterReader pd("/home/abds/系统文件备份/HypothesesParameters.txt");
//	HypoPara hp(pd);
//	ofstream writefile(pd.getData( "svm_directory" ).c_str(),ios::trunc);
//	std::vector<PointCloud::Ptr> model;
//	PointCloud::Ptr model_transform (new PointCloud);
//	for(int i =1 ; i <= 5; i++){
//		PointCloud::Ptr model_temp(new PointCloud);
//		char directory[512];
//		sprintf(directory,"/home/abds/study/stl/%d.pcd",i);
//		pcl::io::loadPCDFile(directory,*model_temp);
//		model.push_back(model_temp);
//	}
//
//
//	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//	tree->setInputCloud (scene_filter);
//
//	std::vector<pcl::PointIndices> cluster_indices;
//	pcl::EuclideanClusterExtraction<PointT> ec;
//	ec.setClusterTolerance (0.02); // 2cm
//	ec.setMinClusterSize (500);
//	ec.setMaxClusterSize (20000);
//	ec.setSearchMethod (tree);
//	ec.setInputCloud (scene_filter);
//	ec.extract (cluster_indices);
//
//
//	std::vector<PointCloud::Ptr> cloud_indices;
//	for(std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();it != cluster_indices.end();++it){
//		PointCloud::Ptr cloud_cluster (new PointCloud);
//		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
//			cloud_cluster->points.push_back (scene_filter->points[*pit]);
//			cloud_cluster->width = cloud_cluster->points.size ();
//			cloud_cluster->height = 1;
//			cloud_cluster->is_dense = true;
//		}
//		cloud_indices.push_back(cloud_cluster);
//
//	}
//	std::cout<<cloud_indices.size()<<std::endl;
//
//
//	std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> vfhvector;
//	for(std::vector<PointCloud::Ptr>::iterator it = cloud_indices.begin();it != cloud_indices.end();++it){
//		cout<<"共"<<(*it)->points.size()<<"点"<<endl;
//
//		/********************************
//		 *
//		 * 测试cvfh
//		 *
//		 ********************************/
//
//		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhfeature(new pcl::PointCloud<pcl::VFHSignature308>);
//		cloud_normals = NormalOMP(*it);
//		vfhfeature = CVFHEst(*it,cloud_normals);
//		vfhvector.push_back(vfhfeature);
//		writefile<<0<<" ";
//		for(int i =0; i < 308;i++){
//			writefile <<i+1<<":"<<vfhfeature->points[0].histogram[i]<<" ";
//		}
//		writefile<<endl;
//	}
//	writefile.close();
//	cout<<vfhvector.size()<<endl;
//	predict_class();
//	std::ifstream fin( pd.getData( "svm_output" ).c_str());
//	std::vector<int> number_vector;
//	while(!fin.eof()){
//		std::string str;
//		std::getline( fin, str );
//		int number;
//		number =atoi(str.c_str());
//		number_vector.push_back(number);
//	}
//	fin.close();
//
//
//	view.removeAllPointClouds(globalpara.vp_2);
//	view.removeAllShapes(globalpara.vp_2);
//	Eigen::Vector4f coordinate_origin(0,0,0,1);
//	Eigen::Vector4f coordinate_x(0.2,0,0,0);
//	Eigen::Vector4f coordinate_y(0,0.2,0,0);
//	Eigen::Vector4f coordinate_z(0,0,0.2,0);
//	coordinate_origin = Camera2Table().inverse()*coordinate_origin;
//	coordinate_x = Camera2Table().inverse()*coordinate_x;
//	coordinate_y = Camera2Table().inverse()*coordinate_y;
//	coordinate_z = Camera2Table().inverse()*coordinate_z;
//	view.addPointCloud (scene, "scene",globalpara.vp_2);
//	view.addLine(pcl::PointXYZ(coordinate_origin[0],coordinate_origin[1],coordinate_origin[2]),pcl::PointXYZ(coordinate_origin[0]+coordinate_x[0],coordinate_origin[1]+coordinate_x[1],coordinate_origin[2]+coordinate_x[2]),1,0,0,"coordinate_x",globalpara.vp_2);
//	view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,"coordinate_x",globalpara.vp_2);
//	view.addLine(pcl::PointXYZ(coordinate_origin[0],coordinate_origin[1],coordinate_origin[2]),pcl::PointXYZ(coordinate_origin[0]+coordinate_y[0],coordinate_origin[1]+coordinate_y[1],coordinate_origin[2]+coordinate_y[2]),0,1,0,"coordinate_y",globalpara.vp_2);
//	view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,"coordinate_y",globalpara.vp_2);
//	view.addLine(pcl::PointXYZ(coordinate_origin[0],coordinate_origin[1],coordinate_origin[2]),pcl::PointXYZ(coordinate_origin[0]+coordinate_z[0],coordinate_origin[1]+coordinate_z[1],coordinate_origin[2]+coordinate_z[2]),0,0,1,"coordinate_z",globalpara.vp_2);
//	view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,"coordinate_z",globalpara.vp_2);
//	std::vector<TargetPose> targetpose_vector;
//	for(size_t i =0;i<cloud_indices.size();i++){
//		Ti = Eigen::Matrix4f::Identity();
//		PointCloud::Ptr object_filter;
//		object_filter = UniformSampling(cloud_indices[i],0.005f);
//		int ModelNumber= number_vector[i]-1;
//		cout<<ModelNumber<<endl;
//		model_transform = myFPFH_Registration(model[ModelNumber],object_filter);
//		for(int j= 0 ; j<20 ;j++){
//			model_transform = ICP_test(model[ModelNumber],model_transform);
//		}
//		Eigen::Matrix4f tf;
//		tf = Ti.inverse();
//		pcl::transformPointCloud(*model[ModelNumber],*model_transform,tf);
//
//		Eigen::Vector4f center;
//		Eigen::Vector4f x_axis(0.1,0,0,0);
//		Eigen::Vector4f y_axis(0,0.1,0,0);
//		Eigen::Vector4f z_axis(0,0,0.1,0);
//
//		pcl::compute3DCentroid(*model_transform,center);
//		x_axis =tf* x_axis;
//		y_axis =tf* y_axis;
//		z_axis =tf* z_axis;
//
//		char name[10];
//		sprintf(name,"%d",i);
//		view.addPointCloud (model_transform,pcl::visualization::PointCloudColorHandlerCustom<PointT> (model_transform,255,0,0),name,globalpara.vp_2);
//		sprintf(name,"x_axis%d",i);
//		view.addLine(pcl::PointXYZ(center[0],center[1],center[2]),pcl::PointXYZ(center[0]+x_axis[0],center[1]+x_axis[1],center[2]+x_axis[2]),1,0,0,name,globalpara.vp_2);
//		view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,name,globalpara.vp_2);
//		sprintf(name,"y_axis%d",i);
//		view.addLine(pcl::PointXYZ(center[0],center[1],center[2]),pcl::PointXYZ(center[0]+y_axis[0],center[1]+y_axis[1],center[2]+y_axis[2]),0,1,0,name,globalpara.vp_2);
//		view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,name,globalpara.vp_2);
//		sprintf(name,"z_axis%d",i);
//		view.addLine(pcl::PointXYZ(center[0],center[1],center[2]),pcl::PointXYZ(center[0]+z_axis[0],center[1]+z_axis[1],center[2]+z_axis[2]),0,0,1,name,globalpara.vp_2);
//		view.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,5,name,globalpara.vp_2);
//
//		pcl::transformPointCloud(*model_transform,*model_transform,Camera2Table());
//		pcl::compute3DCentroid(*model_transform,center);
//		x_axis =Camera2Table()* x_axis;
//		y_axis =Camera2Table()* y_axis;
//		z_axis =Camera2Table()* z_axis;
//		cout<<center<<endl;
//		cout<<ModelNumber<<endl;
//
//		TargetPose targetpose_temp;
//		targetpose_temp.number=ModelNumber;
//		targetpose_temp.centroid=center;
//		targetpose_temp.x_axis=x_axis;
//		targetpose_temp.y_axis=y_axis;
//		targetpose_temp.z_axis=z_axis;
//		targetpose_vector.push_back(targetpose_temp);
//
//
//
//
//		cout<<"第"<<i+1<<"次"<<endl;
//		cout<<"共"<<cloud_indices.size()<<"次"<<endl;
//	}
//
//	stop = time(NULL);
//	cout<<"Use Time:"<<stop-start<<endl;
//
//	int csk;
//	if(socketInit(csk) == -1){
//		printf("socket init fail\n");
//		pthread_exit(0);
//	}
//	prepareformove(csk);
//	Eigen::Vector4f target,corner;
//	Eigen::Matrix4f table2manipulate;
//	corner<<-0.362,0.05,0,1;
//	table2manipulate<<
//			-0.469471,0.882947,0,0.375,
//			-0.882947,-0.469471,0,-0.16,
//			0,0,1,0.26,
//			0,0,0,1;
//	corner = table2manipulate*corner;
//	char biaozhi;
//	HandControl hc;
//	for(std::vector<TargetPose>::iterator it = targetpose_vector.begin(); it != targetpose_vector.end();it++){
//		cout<<"Model "<<it->number+1<<endl;
//
//		switch(it->number){
//			case 3:{
//				Eigen::Vector4f Z;
//				Z<<0,0,1,0;
//				float angle=acos(Z.dot(it->z_axis)/it->z_axis.norm())/PI*180;
//				cout<<angle<<endl;
//				if(abs(angle) <20 || abs(angle) >160){
//					cout<<it->centroid<<endl;
//					cout<<it->x_axis<<endl;
//				}
//				else{
//					cout<<it->centroid<<endl;
//					cout<<it->z_axis<<endl;
//				}
//			}break;
//			case 4:cout<<it->centroid<<endl;break;
//			default:cout<<it->centroid<<endl;
//					cout<<it->z_axis<<endl;
//					break;
//		}
//		target = table2manipulate* it->centroid;
//		socketmove(csk,target[0],target[1],target[2]+0.1);
//		sleep(3);
//		socketmove(csk,target[0],target[1],target[2]);
//		sleep(1);
//		hc.TrianglePick();
//		sleep(2);
//		socketmove(csk,target[0],target[1],target[2]+0.1);
//		sleep(3);
//		socketmove(csk,corner[0],corner[1],target[2]+0.1);
//		sleep(3);
//		socketmove(csk,corner[0],corner[1],target[2]);
//		sleep(1);
//		hc.TriangleOpen();
//		sleep(2);
//		socketmove(csk,corner[0],corner[1],target[2]+0.1);
//		sleep(3);
//	}
//	socketclose(csk);
//	globalpara.thread_over =1;
//	pthread_exit(0);
//}
