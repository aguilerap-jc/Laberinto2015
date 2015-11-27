#include "laberinto.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define _USE_MATH_DEFINES //se necesita para usar PI
#define distanciaPared 0.35
#define giro90Derecha ((-M_PI/2)+0.26) //15 grados
#define giro90Izquierda ((M_PI/2)-0.17)//10 grados
#define giro180Izquierda (M_PI-0.52) //30 grados

RNG rng(12345);
int thresh=40;
float area;
int length;
bool DEBUG = false;
bool mover = true;
vector<vector<Point> > contoursClean;
Mat canny_output;

int getOrientation(vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);//run.cChMaxX(0.5, motion, posture);
    //motion.moveTo(distancia, 0, 0);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    // Draw the principal components
    //circle(img, pos, 3, CV_RGB(255, 0, 255), 2);
    line(img, pos, pos + 0.02 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 255, 0));
    line(img, pos, pos + 0.02 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 255, 255));

    return ((atan2(eigen_vecs[0].y, eigen_vecs[0].x))*180)/M_PI;
}

int getOrientationOnly(vector<Point> &pts)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);//run.cChMaxX(0.5, motion, posture);
    //motion.moveTo(distancia, 0, 0);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    return ((atan2(eigen_vecs[0].y, eigen_vecs[0].x))*180)/M_PI;
}

void aline(AL::ALMotionProxy movimiento, AL::ALMemoryProxy memoria)
{
    float izquierda;
    float derecha;
    bool alinear = true;
    int contador = 6;
    do{
        //Alinear con la pared

        //cout<<"Alinear con la pared 1."<<endl;
        //cout<<"Contador = "<<contador<<endl;
        izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
        //std::cout<<"US Left: "<<izquierda<<std::endl;
        derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
        //std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

        if ((izquierda - derecha > 0.03))//&&(izquierda - derecha < 0.15))
            movimiento.moveTo(0,0,-M_PI/16);
        else if ((derecha - izquierda > 0.03))//&&(derecha - izquierda < 0.15))
            movimiento.moveTo(0,0,M_PI/16);
        else
            alinear = false;

        contador--;
        if (contador == 0)
        {
            movimiento.moveTo(-0.1,0,0);
            contador = 6;
        }
    } while(alinear);

    //std::cout<<"Acercar o Alejar de la pared"<<std::endl;
    izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
    //std::cout<<"US Left: "<<izquierda<<std::endl;
    derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
    // std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

    if((derecha != distanciaPared)&&(izquierda<0.5))
        movimiento.moveTo(derecha - distanciaPared, 0,0);

    alinear = true;
    contador = 6;
    do{

        //Alinear con la pared
        //std::cout<<"Alinear con la pared 2."<<std::endl;
        //cout<<"Contador = "<<contador<<endl;
        izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
        //std::cout<<"US Left: "<<izquierda<<std::endl;
        derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
        //std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;
        //cout<<"Contador = "<<contador<<endl;

        if ((izquierda - derecha > 0.03))//&&(izquierda - derecha < 0.15))
            movimiento.moveTo(0,0,-M_PI/16);
        else if ((derecha - izquierda > 0.03))//&&(derecha - izquierda < 0.15))
            movimiento.moveTo(0,0,M_PI/16);
        else
            alinear = false;

        contador--;
    }while(alinear || contador == 0);
}

void buscarNaomark(AL::ALMotionProxy movimiento, AL::ALMemoryProxy memoria, AL::ALLandMarkDetectionProxy naoMark, AL::ALTextToSpeechProxy say)
{

    int contador;
    bool detected ;

    AL::ALValue markInfo = "";

    contador = 0;
    detected = false;
    do{
        markInfo = memoria.getData("LandmarkDetected");
        contador++;
        if (markInfo.getSize()!=0)
            detected = true;
    }while(contador < 10 && !detected);

    if(markInfo.getSize() != 0) {
        std::string markID = "";
        markID = markInfo[1][0][1].toString().substr(1,3);
        say.say(markID);
        std::cout << "markID = " << markID << std::endl;
    }
    else
    {
        std::cout << "No se detectó marca enfrente" << std::endl;

        movimiento.angleInterpolation("HeadYaw",M_PI/2, 1.0 ,true);

        usleep(1000000);

        contador = 0;
        detected = false;
        do{
            markInfo = memoria.getData("LandmarkDetected");
            contador++;
            if (markInfo.getSize()!=0)
                detected = true;
        }while(contador < 10 && !detected);

        if(markInfo.getSize() != 0) {
            std::string markID = "";
            markID = markInfo[1][0][1].toString().substr(1,3);
            say.say(markID);
            std::cout << "markID = " << markID << std::endl;
            movimiento.moveTo(0,0, giro90Izquierda-0.21);
        }
        else
        {
            std::cout << "No se detectó marca a la izquierda" << std::endl;
            movimiento.angleInterpolation("HeadYaw",-M_PI/2, 1.0 ,true);

            usleep(1000000);

            contador = 0;
            detected = false;
            do{
                markInfo = memoria.getData("LandmarkDetected");
                contador++;
                if (markInfo.getSize()!=0)
                    detected = true;
            }while(contador < 10  && !detected);

            if(markInfo.getSize() != 0) {
                std::string markID = "";
                markID = markInfo[1][0][1].toString().substr(1,3);
                say.say(markID);
                std::cout << "markID = " << markID << std::endl;
                movimiento.moveTo(0,0, giro90Derecha);
            }
            else
            {
                std::cout << "No se detectó a la derecha" << std::endl;
            }
        }
        movimiento.angleInterpolation("HeadYaw",0, 1.0 ,true);
    }
}

void checarSideCamara( AL::ALVideoDeviceProxy camProxy, std::string clientName, ALMotionProxy movimiento, vector<vector<Point> > contours, vector<Vec4i> hierarchy)
{

    Mat src, src_gray;
    //Get image from NAO
    camProxy.setActiveCamera(1 ); //conect to bottom camera
    camProxy.setResolution("test", 1);
    /** Create an cv::Mat header to  wrap into an opencv image.*/
    Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

    /** Create a OpenCV window to display the images. */
    //cv::namedWindow("images");

    ALValue img = camProxy.getImageRemote(clientName);

    /** Access the image buffer (6th field) and assign it to the opencv image
    * container. */
    imgHeader.data = (uchar*) img[6].GetBinary();
    /** Tells to ALVideoDevice that it can give back the image buffer to the
    * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
    camProxy.releaseImage(clientName);
    /** Display the iplImage on screen.*/
    src = imgHeader.clone();
    //imshow("src", src);

    /// Convert image to gray and blur it
    cvtColor( src, src_gray, CV_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );
    //imshow( "src", src );
    //waitKey(500);

    /// Detect edges using canny
    Canny( src_gray, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }

    ///  Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }

    //Eliminate contours without Area
    contoursClean.clear();
    int indMaxSide = 0;
    int lengthMaxSide = 0;
    Point2f punto;
    for(int i = 0; i < contours.size(); i++) {
        area = mu[i].m00;
        length = arcLength(contours[i], true);
        punto = mc[i];
        if(DEBUG){
            cout << "Area " << i << " = " << area << endl;
            cout << "Length " << i << " = " << length << endl;
            cout << "mass Center " << i << " = " << punto.x << "," << punto.y << endl;
            cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
        }
        if(area != 0 && length > 100 && punto.x > 0 && punto.y > 0) {
            contoursClean.push_back(contours.at(i));
        }
    }
    if(contoursClean.size() != 0) {
        //Get moments and mass for new vector
        vector<Moments> muClean(contoursClean.size() );
        for( int i = 0; i < contoursClean.size(); i++ )
        { muClean[i] = moments( contoursClean[i], false ); }

        ///  Get the mass centers:
        vector<Point2f> mcClean( contoursClean.size() );
        for( int i = 0; i < contoursClean.size(); i++ ) {
            mcClean[i] = Point2f( muClean[i].m10/muClean[i].m00 , muClean[i].m01/muClean[i].m00 );
        }

        for(int i = 0; i < contoursClean.size(); i++) {
            punto = mcClean[i];
            length = arcLength(contoursClean[i], true);
            if(DEBUG){
                cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
                cout << "Area " << i << " = " << area << endl;
                cout << "Length " << i << " = " << length << endl;
                cout << "mass Center " << i << " = " << punto.x << "," << punto.y << endl;
                cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
            }
        }
        //Encontrar el mas largo
        for(int i = 0; i < contoursClean.size(); i++) {
            length = arcLength(contoursClean[i], true);
            lengthMaxSide = arcLength(contoursClean[indMaxSide], true);
            int angle = getOrientationOnly(contoursClean[i]);
            if((i > 0)&&((angle > 30)||(angle < -30))|| (angle < 90) || (angle > -90)) {
                if(length  > lengthMaxSide) {
                    indMaxSide = i;
                }
            } else {
                indMaxSide = 0;
            }
        }
        /// Draw contours
        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        cout << "*************************************" << endl;
        cout << "# de lineas encontradas = " << contoursClean.size() << endl;
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( drawing, contoursClean, indMaxSide, color, 2, 8, hierarchy, 0, Point() );
        //circle( drawing, mcClean[indMaxSide], 4, color, 5, 8, 0 );

        //Orientation
        int orientationSide = getOrientation(contoursClean[indMaxSide], drawing);
        cout << "orientationSide degrees = " << orientationSide <<endl;
        //}

        /// Show in a window

        Point2f puntoMaxSide;
        if(contoursClean.size() != 0) {

            //line(drawing, Point((drawing.cols/10) ,0), Point((drawing.cols/10) , drawing.rows), Scalar(255,255,255));
           // line(drawing, Point((drawing.cols*2/10) ,0), Point((drawing.cols*2/10) , drawing.rows), Scalar(255,255,255));
            //line(drawing, Point((drawing.cols*8/10) ,0), Point((drawing.cols*8/10) , drawing.rows), Scalar(255,255,255));
           // line(drawing, Point((drawing.cols*9/10) ,0), Point((drawing.cols*9/10) , drawing.rows), Scalar(255,255,255));
            //namedWindow( "Side orientationSide", CV_WINDOW_AUTOSIZE );
            //imshow( "Side orientationSide", drawing );
            //waitKey(50);

            puntoMaxSide = mcClean[indMaxSide];
            lengthMaxSide = arcLength(contoursClean[indMaxSide], true);
            cout << "El mas largo de lado es el " << indMaxSide << " con " << lengthMaxSide << endl;
            cout << "Y esta en la posicion x = " << puntoMaxSide.x << " y = " << puntoMaxSide.y << endl;
            cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;


            if(puntoMaxSide.x > (drawing.cols*5/10) && puntoMaxSide.x < (drawing.cols*9/10) &&  ((orientationSide > 30 && orientationSide <= 90)||(orientationSide < -110)) )  { //linea correcta
                cout << "Side Corregir a la Izquierdaaaa" << endl;
                movimiento.moveTo(0,0, M_PI/16);
            } else if(puntoMaxSide.x > (drawing.cols*1/10) && puntoMaxSide.x < (drawing.cols*9/10)  && ((orientationSide < -30 && orientationSide > -90) || (orientationSide > 110))) { //linea correcta
                cout << "Side Corregir a la Derechaaaa" << endl;
                movimiento.moveTo(0,0, -M_PI/16);
            } else {
                cout << "Side No corregir111111" << endl;

            }
        } else {
            cout << "Side No hay lineas detectadas" << endl;
            cout << "Side Sigue Derecho222222" << endl;
        }
    } else {
        cout << "Side No hay lineas detectadas" << endl;
        cout << "Sigue Derecho3333" << endl;
    }
}



laberinto::laberinto()
{
    thresh = 40;
    ejecutar = false;
}

void laberinto::resolver()
{
    AL::ALSonarProxy sonar;
    AL::ALMemoryProxy memoria;
    AL::ALRobotPostureProxy posture;
    AL::ALMotionProxy movimiento;
    AL::ALLandMarkDetectionProxy naoMark;
    AL::ALTextToSpeechProxy say;
    AL::ALVideoDeviceProxy camProxy;
    sonar.subscribe("ALSonar");
    bool stand;

    std::string  postura;
    stand = posture.goToPosture("Stand",1);


    int period = 500;
    naoMark.subscribe("Test_Mark", period, 0.0);

    const std::string clientName = camProxy.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 30);
    Mat src, src_gray;

    float izquierda;
    float derecha;


    bool girar90 = false;
    bool giro180 = false;

    if (mover)
        movimiento.moveTo(0.2,0,0);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    while(ejecutar)
    {

        //Get image from NAO
        camProxy.setActiveCamera(1 ); //conect to bottom camera
        camProxy.setResolution("test", 1);

        /** Create an cv::Mat header to  wrap into an opencv image.*/
        Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

        /** Create a OpenCV window to display the images. */
        //cv::namedWindow("images");

        ALValue img = camProxy.getImageRemote(clientName);

        /** Access the image buffer (6th field) and assign it to the opencv image
        * container. */
        imgHeader.data = (uchar*) img[6].GetBinary();
        /** Tells to ALVideoDevice that it can give back the image buffer to the
        * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
        camProxy.releaseImage(clientName);
        /** Display the iplImage on screen.*/
        src = imgHeader.clone();
        //imshow("src", src);

        /// Convert image to gray and blur it
        ////imshow("srcBGray", src);
        ////waitKey(50);
        cvtColor( src, src_gray, CV_BGR2GRAY );
        blur( src_gray, src_gray, Size(3,3) );
        ////imshow("blur", src_gray);
        //imshow( "src", src );
        //waitKey(500);

        /// Detect edges using canny
        Canny( src_gray, canny_output, thresh, thresh*2, 3 );
        /// Find contours
        findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);

        //////////////////////////////////////////////////////////////////////////
        postura = posture.getPostureFamily();
        std::cout<<postura<<std::endl;

        if ((postura != "Standing"))
        {
            posture.goToPosture("Stand",1);
            //buscar pared izquierda para alinearse
            bool alinear = true;
            do{

                //Alinear con la pared
                izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                std::cout<<"US Left: "<<izquierda<<std::endl;
                derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                if ((izquierda - derecha > 0.10))//&&(izquierda - derecha < 0.15))
                    movimiento.moveTo(0,0,M_PI/16);
                else if ((derecha - izquierda > 0.10))//&&(derecha - izquierda < 0.15))
                    movimiento.moveTo(0,0,M_PI/16);
                else
                    alinear = false;

            } while(alinear);
            aline(movimiento, memoria);

        }


        if (!giro180 && ejecutar)
        {
            camProxy.setActiveCamera(0); //conect to top camera
            buscarNaomark(movimiento,memoria,naoMark,say);
        }

        if (mover && ejecutar)
        {
            izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
            std::cout<<"US Left: "<<izquierda<<std::endl;
            derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
            std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

            if ((derecha < 1.2 && izquierda < 1.2) && (derecha > 0.65 && izquierda > 0.65)&& ejecutar)//antes 1.4
            {
                //Avanzar para colocarse a 20 de la pared
                std::cout<<"Pared a menos de 1.2m, quedar a 32cm de la pared"<<std::endl;

                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                movimiento.moveTo(0.2,0,0);
                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                movimiento.moveTo(0.2,0,0);
                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                movimiento.moveTo(0.2,0,0);

                izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                std::cout<<"US Left: "<<izquierda<<std::endl;
                derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                movimiento.moveTo(derecha - distanciaPared,0,0);
                aline(movimiento, memoria);
                giro180=false;
            }
            else if (derecha > 0.9 && izquierda > 0.9 && ejecutar)
            {
                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                std::cout<<"Avanzar 60 centimetros y checar si hay pared"<<std::endl;
                movimiento.moveTo(0.2,0,0);
                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                movimiento.moveTo(0.2,0,0);
                checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                movimiento.moveTo(0.2,0,0);
                giro180=false;

                //Girar 90º derecha
                movimiento.moveTo(0,0, giro90Derecha);

                izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                std::cout<<"US Left: "<<izquierda<<std::endl;
                derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                if ((izquierda < 0.5 && derecha < 0.5 )&& ejecutar) //Si hay pared
                {
                    aline(movimiento, memoria);
                    movimiento.moveTo(0,0,giro90Izquierda -0.21 );
                }
            }
            else if (derecha < 0.5 && izquierda < 0.5 && ejecutar)
            {
                std::cout<<"Pared a menos de 50 cm, quedar a 32cm de la pared"<<std::endl;
                movimiento.moveTo(derecha - distanciaPared,0,0);
                aline(movimiento, memoria);
                std::cout<<"Girar 90 grados a la derecha"<<std::endl;
                movimiento.moveTo(0,0, giro90Derecha);


                izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                std::cout<<"US Left: "<<izquierda<<std::endl;
                derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                //Si hay pared a menos de 70, girar 180 porque no es el camino.
                if (derecha < 0.70 && izquierda < 0.70 && ejecutar)
                {
                    std::cout<<"Hubo pared a la derecha."<<std::endl;
                    aline(movimiento, memoria);
                    std::cout<<"Girar 180 grados"<<std::endl<<std::endl;
                    movimiento.moveTo(0,0, giro90Izquierda);
                    aline(movimiento, memoria);
                    movimiento.moveTo(0,0, giro90Izquierda -0.21); //bajarle al valor
                    std::cout<<"Terminó de girar los 180."<<std::endl;
                    giro180 = true;

                    izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                    std::cout<<"US Left: "<<izquierda<<std::endl;
                    derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                    std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                    //Si hay pared entonces es un callejon y tiene que girar 90 grados a la izquierda para regresar
                    if (derecha < 0.70 && izquierda < 0.70 && ejecutar)
                    {
                        aline(movimiento, memoria);
                        //std::cout<<"Girar 90 grados a la izquierda"<<std::endl<<std::endl;
                        std::cout<<"Salí de callejón."<<std::endl<<std::endl;
                        movimiento.moveTo(0,0,giro90Izquierda-0.21);
                        giro180=false;
                    }
                }
            }
            else if (ejecutar)
            {
                girar90 = false;
                /// Get the moments
                vector<Moments> mu(contours.size() );
                for( int i = 0; i < contours.size(); i++ )
                { mu[i] = moments( contours[i], false ); }

                ///  Get the mass centers:
                vector<Point2f> mc( contours.size() );
                for( int i = 0; i < contours.size(); i++ ) {
                    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                }

                //Eliminate contours without Area
                contoursClean.clear();
                int indMaxSide = 0;
                int lengthMaxSide = 0;
                Point2f punto;
                for(int i = 0; i < contours.size(); i++) {
                    area = mu[i].m00;
                    length = arcLength(contours[i], true);
                    punto = mc[i];
                    if(DEBUG){
                        cout << "Area " << i << " = " << area << endl;
                        cout << "Length " << i << " = " << length << endl;
                        cout << "mass Center " << i << " = " << punto.x << "," << punto.y << endl;
                        cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
                    }
                    if(area != 0 && length > 100 && punto.x > 0 && punto.y > 0) {
                        contoursClean.push_back(contours.at(i));
                    }
                }
                if(contoursClean.size() != 0) {
                    //Get moments and mass for new vector
                    vector<Moments> muClean(contoursClean.size() );
                    for( int i = 0; i < contoursClean.size(); i++ )
                    { muClean[i] = moments( contoursClean[i], false ); }

                    ///  Get the mass centers:
                    vector<Point2f> mcClean( contoursClean.size() );
                    for( int i = 0; i < contoursClean.size(); i++ ) {
                        mcClean[i] = Point2f( muClean[i].m10/muClean[i].m00 , muClean[i].m01/muClean[i].m00 );
                    }

                    for(int i = 0; i < contoursClean.size(); i++) {
                        punto = mcClean[i];
                        length = arcLength(contoursClean[i], true);
                        if(DEBUG){
                            cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
                            cout << "Area " << i << " = " << area << endl;
                            cout << "Length " << i << " = " << length << endl;
                            cout << "mass Center " << i << " = " << punto.x << "," << punto.y << endl;
                            cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;
                        }
                    }
                    //Encontrar el mas largo
                    for(int i = 0; i < contoursClean.size(); i++) {
                        length = arcLength(contoursClean[i], true);
                        lengthMaxSide = arcLength(contoursClean[indMaxSide], true);
                        int angle = getOrientationOnly(contoursClean[i]);
                        if((i > 0)&&((angle > 30)||(angle < -30))|| (angle < 90) || (angle > -90)) {
                            if(length  > lengthMaxSide) {
                                indMaxSide = i;
                            }
                        } else {
                            indMaxSide = 0;
                        }


                    }
                    /// Draw contours
                    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
                    cout << "*************************************" << endl;
                    cout << "# de lineas encontradas = " << contoursClean.size() << endl;
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    //drawContours( drawing, contoursClean, indMaxSide, color, 2, 8, hierarchy, 0, Point() );
                    //circle( drawing, mcClean[indMaxSide], 4, color, 5, 8, 0 );

                    //Orientation
                    int orientationSide = getOrientation(contoursClean[indMaxSide], drawing);
                    cout << "orientationSide Main degrees = " << orientationSide <<endl;

                    /// Show in a window

                    Point2f puntoMaxSide;
                    if(contoursClean.size() != 0) {

                        line(drawing, Point((drawing.cols/10) ,0), Point((drawing.cols/10) , drawing.rows), Scalar(255,255,255));
                        line(drawing, Point((drawing.cols*8/10) ,0), Point((drawing.cols*8/10) , drawing.rows), Scalar(255,255,255));
                        //namedWindow( "Main", CV_WINDOW_AUTOSIZE );
                        //imshow( "Main", drawing );
                        //waitKey(50);


                        puntoMaxSide = mcClean[indMaxSide];
                        lengthMaxSide = arcLength(contoursClean[indMaxSide], true);
                        cout << "El mas largo de lado es el " << indMaxSide << " con " << lengthMaxSide << endl;
                        cout << "Y esta en la posicion x = " << puntoMaxSide.x << " y = " << puntoMaxSide.y << endl;
                        cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;

                        //waitKey(50);

                        if(puntoMaxSide.x > (drawing.cols*3/10) && puntoMaxSide.x < (drawing.cols*8/10) &&  orientationSide > 20 && orientationSide <= 90 )  { //linea correcta
                            cout << "Main Corregir a la Izquierdaaaa" << endl;
                            movimiento.moveTo(0,0, M_PI/16);
                        } else if(puntoMaxSide.x > (drawing.cols*1/10) && puntoMaxSide.x < (drawing.cols*8/10)  && orientationSide < -20 && orientationSide > -90) { //linea correcta
                            cout << "Main Corregir a la Derechaaaa" << endl;
                            movimiento.moveTo(0,0, -M_PI/16);
                        } else {
                            cout << "Main No corregir111111" << endl;
                            girar90 = true;
                            giro180 = true;
                            movimiento.moveTo(0.2,0,0);
                            //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                            checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                            movimiento.moveTo(0.2,0,0);
                            //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                            checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                            movimiento.moveTo(0.1,0,0);

                        }

                    } else {
                        cout << "No hay lineas detectadas" << endl;
                        cout << "Sigue Derecho222222" << endl;
                        movimiento.moveTo(0.2,0,0);
                        //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                        checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                        movimiento.moveTo(0.2,0,0);
                        //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                        checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                        movimiento.moveTo(0.1,0,0);
                        girar90=true;
                        giro180=true;
                    }
                } else {
                    cout << "No hay lineas detectadas" << endl;
                    cout << "Sigue Derecho3333" << endl;
                    movimiento.moveTo(0.2,0,0);
                    //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                    checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);

                    movimiento.moveTo(0.2,0,0);
                    //////checarMuroCamara(camProxy, clientName, movimiento, contours, hierarchy);
                    checarSideCamara(camProxy, clientName, movimiento, contours, hierarchy);
                    movimiento.moveTo(0.12,0,0);
                    girar90=true;
                    giro180=true;
                }

                if (girar90 && ejecutar)
                {
                    movimiento.moveTo(0,0,giro90Derecha);
                    izquierda = memoria.getData("Device/SubDeviceList/US/Left/Sensor/Value");
                    std::cout<<"US Left: "<<izquierda<<std::endl;
                    derecha = memoria.getData("Device/SubDeviceList/US/Right/Sensor/Value");
                    std::cout<<"US Right: "<<derecha<<std::endl<<std::endl<<std::endl;

                    if (izquierda < 0.75 && derecha < 0.75 ) //Si hay pared
                    {
                        aline(movimiento, memoria);
                        movimiento.moveTo(0,0,giro90Izquierda -0.21 );
                    }
                }

            }
        }
    }

    camProxy.unsubscribe(clientName);

    posture.goToPosture("Crouch",1);

    movimiento.setStiffnesses("Body", 0);
}

void laberinto::setEjecutar(bool valor){

    ejecutar = valor;
}

