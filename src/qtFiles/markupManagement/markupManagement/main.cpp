#include <QtGui/QApplication>
#include "markupmanagementdialog.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MarkupManagementDialog w;
    w.show();
    
    return a.exec();
}
