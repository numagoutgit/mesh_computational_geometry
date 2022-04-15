#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::displayWireframe() {
    ui->widget->wireFrame = not ui->widget->wireFrame;
};

void MainWindow::displayLaplacian() {
    ui->widget->laplacian = not ui->widget->laplacian;
};
