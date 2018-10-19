#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>

#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include "custom_msgs/GeoImageCompressed.h"


#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <exiv2/exiv2.hpp>


namespace fs = boost::filesystem;
// TODO:: Constant Heading and Altitude from launch file.

std::vector<std::string> 	get_file_list(const std::string& path);
void 						Publish(std::string& aImagePath, ros::Publisher& aPublisher);
void 						PublishGeoImage(std::string& aImagePath, ros::Publisher& aPublisher);
sensor_msgs::NavSatFix 		exivStringToRosGPS( Exiv2::ExifData & aExifData );
std::vector<double> 		readXmpDataFromImage(Exiv2::XmpData & aXmpData );
double 						degMinSecToDecimal(double aArray[3]);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_from_images_node");
	ros::NodeHandle lnh,l_pv_nh("~");

    std::string l_image_path,l_image_topic,l_geo_image_topic;
    float l_freq, l_altitude, l_heading;

    l_pv_nh.param("image_path", l_image_path, std::string("uninitialised"));
    l_pv_nh.param("image_topic", l_image_topic, std::string("uninitialised"));
    l_pv_nh.param("geoimage_topic", l_geo_image_topic, std::string("uninitialised"));
    l_pv_nh.param("lFreq", l_freq, float(0));
    l_pv_nh.param("altitude", l_altitude, float(-1000));
    l_pv_nh.param("heading", l_heading, float(-1000));


    // Read the files into the list.
    std::vector<std::string> l_image_list = get_file_list(l_image_path);
    std::vector<std::string> l_current_img_list, l_new_imgages_list;
    std::sort(l_image_list.begin(),l_image_list.end());


    ROS_INFO("File input Image Path is: %s", l_image_path.c_str());
    ROS_INFO("Image publish topic is: %s", l_image_topic.c_str());
    ROS_INFO("GeoImage publish topic is: %s", l_geo_image_topic.c_str());
    ROS_INFO("Image publish frequency is: %f", l_freq);
    std::cout << "No of Input images are: " << l_image_list.size() << std::endl;

    ros::Publisher image_pub = lnh.advertise<sensor_msgs::Image>(l_image_topic, 2);
    ros::Publisher geo_image_pub = lnh.advertise<custom_msgs::GeoImageCompressed>(l_geo_image_topic, 2);
    
    ros::Rate loop_rate(l_freq);
    
    int count = 0;
    int rosspincounter = 0;
    while (ros::ok() )
    {
    	//- Process the image list.
    	if ( count < l_image_list.size() )
    	{
    		ROS_INFO_STREAM("Image No: " << count << "Image Path: " << l_image_list[count]);
    		Publish(l_image_list[count], image_pub);
        	PublishGeoImage(l_image_list[count], geo_image_pub);
        	count++;
        	ROS_INFO_STREAM("Count is: " << count);
    	}

    	//- If all images processed, check for new images and add to the l_image_list
    	if ( count == l_image_list.size() && (rosspincounter % 10 == 0 ) )
    	{
    		ROS_INFO_STREAM("All old images processed. Checking for new images now.");
    		l_current_img_list.clear();
    		l_new_imgages_list.clear();
    		l_current_img_list = get_file_list(l_image_path);
    		ROS_INFO_STREAM("Image List size old and new are. " << l_image_list.size() << " : " \
    			<< l_current_img_list.size() );
    		if (l_image_list.size() != l_current_img_list.size())
    		{
    			std::vector<std::string> tmpHolder(l_image_list);
    			std::sort(tmpHolder.begin(),tmpHolder.end());
    			std::sort(l_current_img_list.begin(),l_current_img_list.end());
    			std::set_difference(l_current_img_list.begin(), l_current_img_list.end(), \
    				tmpHolder.begin(), tmpHolder.end(), std::back_inserter(l_new_imgages_list));
    			ROS_INFO_STREAM("No. of new images: " << l_new_imgages_list.size() );
    			std::sort(l_new_imgages_list.begin(),l_new_imgages_list.end());
    			l_image_list.insert(l_image_list.end(), l_new_imgages_list.begin(), l_new_imgages_list.end());
    			tmpHolder.clear();
    		}
    		ROS_INFO("No new images. Continuing processing.");
    	}
    	ros::spinOnce();
    	rosspincounter++;
    	loop_rate.sleep();
    }
    return 0;
}

void Publish(std::string& aImagePath, ros::Publisher& aPublisher)
{
    cv::Mat im = cv::imread(aImagePath,CV_LOAD_IMAGE_UNCHANGED);
    cv_bridge::CvImage cvImage;
    cvImage.image = im;
    cvImage.encoding = sensor_msgs::image_encodings::RGB8;
    //cvImage.header.stamp = ros::Time::now();
    aPublisher.publish(cvImage.toImageMsg());
}

void PublishGeoImage(std::string& aImagePath, ros::Publisher& aPublisher)
{

    if (aImagePath.empty()) 
    {
        ROS_INFO("No Image found.");
        return;
    }
    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(aImagePath);
    sensor_msgs::NavSatFix gpsdata;
    std::vector <double> xmpdat;
    if (image.get() != 0)
    {
        image->readMetadata();
        Exiv2::ExifData &exifData = image->exifData();
        Exiv2::XmpData &xmpData = image->xmpData();
        if (exifData.empty()) 
        {
            std::string error(aImagePath);
            error += ": No Exif data found in the file";
            std::cout << error << std::endl;
        }
        std::vector<std::string> vect(6);
        vect[2] = exifData["Exif.GPSInfo.GPSLatitudeRef"].toString();        
        vect[3] = exifData["Exif.GPSInfo.GPSLatitude"].toString();            
        vect[4] = exifData["Exif.GPSInfo.GPSLongitudeRef"].toString();            
        vect[5] = exifData["Exif.GPSInfo.GPSLongitude"].toString();            
        vect[0] = exifData["Exif.GPSInfo.GPSAltitudeRef"].toString();            
        vect[1] = exifData["Exif.GPSInfo.GPSAltitude"].toString();
        float alt = exifData["Exif.GPSInfo.GPSLongitude"].toFloat(2);

        std::cout <<  " Image is: " << aImagePath << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSLatitudeRef:  "  << vect[2] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSLatitude:  "     << vect[3] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSLongitudeRef:  " << vect[4] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSLongitude:  "    << vect[5] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSAltitudeRef:  "  << vect[0] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSAltitude:  "     << vect[1] << std::endl;
        std::cout <<  "Exif.GPSInfo.GPSAltitude:  "     << alt << std::endl;
        
        gpsdata = exivStringToRosGPS( exifData );
        xmpdat = readXmpDataFromImage( xmpData );

        std::cout << "\n        Lat : Long: Alt: RelAlt: Heading" << std::endl;
        std::cout << "        " << gpsdata.latitude << " : " << gpsdata.longitude << " : " \
              << gpsdata.altitude << " : " << xmpdat[0] << " : " << xmpdat[1] << "\n" << std::endl;


    }
    custom_msgs::GeoImageCompressed geoCompressedImagemsg_;

    cv::Mat im = cv::imread(aImagePath,CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage cvImage;
    cvImage.image = im;
    cvImage.encoding = sensor_msgs::image_encodings::RGB8;

    geoCompressedImagemsg_.imagedata.format = "bgr8";
    //geoCompressedImagemsg_.imagename.data=image_name_.substr(save_location_.length());
    geoCompressedImagemsg_.imagedata = *cvImage.toCompressedImageMsg();
    geoCompressedImagemsg_.gpsdata=gpsdata;
    geoCompressedImagemsg_.heading.data = xmpdat[1];
    geoCompressedImagemsg_.baroHeight.data = xmpdat[0];
    geoCompressedImagemsg_.header.stamp = ros::Time::now();
    //geoCompressedImagemsg_.missioncount.data = mMissionCount;
    aPublisher.publish(geoCompressedImagemsg_);

    
}

std::vector<std::string> get_file_list(const std::string& path)
{
    std::vector<std::string> m_file_list;
    if (!path.empty())
    {
        fs::path apk_path(path);
        fs::recursive_directory_iterator end;

        for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const fs::path cp = (*i);
            m_file_list.push_back(cp.string());
        }
    }
    return m_file_list;
}


sensor_msgs::NavSatFix exivStringToRosGPS(Exiv2::ExifData & aExifData )
{
    // TODO: Based on N or S, E or W and ASL or BSL calculate the values.
    sensor_msgs::NavSatFix gps_;
    // Altitude
    // if ( vect[1] = "0" ) vect[1] = "0";      // Above Sea Level
    // else 
    // vect[2] =  std::to_string((int) floor(fabs(gps_.altitude))) + "/1";   
    gps_.altitude = aExifData["Exif.GPSInfo.GPSAltitude"].toFloat();


    // Latitude
    double lLatitudeArr[3];
    //if ( gps_.latitude >= 0.0 ) vect[3] = "N";  // Above Equator
    //else vect[3] = "S";
    lLatitudeArr[0] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(0);
    lLatitudeArr[1] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(1);
    lLatitudeArr[2] = aExifData["Exif.GPSInfo.GPSLatitude"].toFloat(2);

    gps_.latitude = degMinSecToDecimal(lLatitudeArr);    

    // Longitude
    double lLongitudeArr[3];
    //if ( gps_.longitude >= 0.0 ) vect[5] = "E";     // East of green meridian
    //else vect[5] = "W";
    lLongitudeArr[0] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(0);
    lLongitudeArr[1] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(1);
    lLongitudeArr[2] = aExifData["Exif.GPSInfo.GPSLongitude"].toFloat(2);

    gps_.longitude = degMinSecToDecimal(lLongitudeArr);

    return gps_;
}

double degMinSecToDecimal(double aArray[3])
{
    double result;
    double angleDeg = aArray[0];
    double angleMin = aArray[1]/60;
    double angleSec = aArray[2]/3600;
    result = angleDeg + angleMin + angleSec;
    return result;
}

std::vector <double> readXmpDataFromImage(Exiv2::XmpData & aXmpData )
{
    std::vector <double> result(2);
    result[0] = aXmpData["Xmp.drone-dji.RelativeAltitude"].toFloat();
    result[1] = aXmpData["Xmp.drone-dji.FlightYawDegree"].toFloat();

    return result;
}