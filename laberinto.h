#ifndef LABERINTO_H
#define LABERINTO_H

#include <alproxies/alsonarproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>

using namespace AL;

class laberinto
{
public:
    laberinto();

    void resolver();

    void setEjecutar(bool valor);

private:
    bool ejecutar;

};

#endif // LABERINTO_H
