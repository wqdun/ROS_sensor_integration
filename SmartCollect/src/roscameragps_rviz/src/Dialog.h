#ifndef DIALOG_H
#define DIALOG_H
#include <QDialog>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QString>
#include <ros/ros.h>

class Dialog : public QDialog
{
public:
    QPushButton ModalBtn;
    QLabel*      lable_cnt;
    QLineEdit*   line_edit;

protected Q_SLOTS:
public:
    QSize sizeHint() const;
public:
    Dialog(QWidget *parent = 0);
    ~Dialog();
};
#endif // DIALOG_H
