// receiver_json.cpp
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
// #include <Qt6/QtCore/QApplicationStatic>
#include <QApplication>
//

#include "Display.h"
using json = nlohmann::json;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Displayer w;
    // w.dumpObjectInfo();
    w.show();
    return a.exec();
}
