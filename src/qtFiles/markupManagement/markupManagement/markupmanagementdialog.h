#ifndef MARKUPMANAGEMENTDIALOG_H
#define MARKUPMANAGEMENTDIALOG_H

#include <QDialog>

namespace Ui {
class MarkupManagementDialog;
}

class MarkupManagementDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit MarkupManagementDialog(QWidget *parent = 0);
    ~MarkupManagementDialog();
    
private:
    Ui::MarkupManagementDialog *ui;
};

#endif // MARKUPMANAGEMENTDIALOG_H
