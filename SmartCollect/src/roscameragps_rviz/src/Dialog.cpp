#include "Dialog.h"
#include <QDebug>
#include <QLineEdit>

Dialog::Dialog(QWidget *parent) : QDialog(parent), ModalBtn(this)
{
    this->sizeHint();
    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowMaximizeButtonHint | Qt::WindowStaysOnTopHint);

    line_edit = new QLineEdit(this);
}
QSize Dialog::sizeHint() const
{
     return QSize( 140, 80 );
}

Dialog::~Dialog()
{
}
