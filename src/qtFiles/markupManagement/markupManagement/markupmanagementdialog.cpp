#include "markupmanagementdialog.h"
#include "ui_markupmanagementdialog.h"

MarkupManagementDialog::MarkupManagementDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MarkupManagementDialog)
{
    ui->setupUi(this);
}

MarkupManagementDialog::~MarkupManagementDialog()
{
    delete ui;
}
