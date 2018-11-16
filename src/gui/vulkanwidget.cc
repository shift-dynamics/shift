/****************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "shift-gui/vulkanwidget.h"
#include <QApplication>
#include <QFileDialog>
#include <QLCDNumber>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QVulkanFunctions>

MainWindow::MainWindow(VulkanWindow* w, QPlainTextEdit* logWidget)
  : window_(w) {
  QWidget* wrapper = QWidget::createWindowContainer(w);

  info_ = new QPlainTextEdit;
  info_->setReadOnly(true);

  number_ = new QLCDNumber(3);
  number_->setSegmentStyle(QLCDNumber::Filled);

  QPushButton* grabButton = new QPushButton(tr("&Grab"));
  grabButton->setFocusPolicy(Qt::NoFocus);

  connect(grabButton, &QPushButton::clicked, this,
          &MainWindow::onGrabRequested);

  QPushButton* quitButton = new QPushButton(tr("&Quit"));
  quitButton->setFocusPolicy(Qt::NoFocus);

  connect(quitButton, &QPushButton::clicked, qApp, &QCoreApplication::quit);

  QVBoxLayout* layout = new QVBoxLayout;
  infoTab_ = new QTabWidget(this);
  infoTab_->addTab(info_, tr("Vulkan Info"));
  infoTab_->addTab(logWidget, tr("Debug Log"));
  layout->addWidget(infoTab_, 2);
  layout->addWidget(number_, 1);
  layout->addWidget(wrapper, 5);
  layout->addWidget(grabButton, 1);
  layout->addWidget(quitButton, 1);
  setLayout(layout);
}

void MainWindow::onVulkanInfoReceived(const QString& text) {
  info_->setPlainText(text);
}

void MainWindow::onFrameQueued(int colorValue) {
  number_->display(colorValue);
}

void MainWindow::onGrabRequested() {
  if(!window_->supportsGrab()) {
    QMessageBox::warning(this, tr("Cannot grab"),
                         tr("This swapchain does not support readbacks."));
    return;
  }

  QImage img = window_->grab();

  // Our startNextFrame() implementation is synchronous so img is ready to be
  // used right here.

  QFileDialog fd(this);
  fd.setAcceptMode(QFileDialog::AcceptSave);
  fd.setDefaultSuffix("png");
  fd.selectFile("test.png");
  if(fd.exec() == QDialog::Accepted)
    img.save(fd.selectedFiles().first());
}

QVulkanWindowRenderer* VulkanWindow::createRenderer() {
  return new VulkanRenderer(this);
}

VulkanRenderer::VulkanRenderer(VulkanWindow* w)
  : TriangleRenderer(w) {}

void VulkanRenderer::initResources() {
  TriangleRenderer::initResources();

  QVulkanInstance* inst = window_->vulkanInstance();
  devFuncs_ = inst->deviceFunctions(window_->device());

  QString info;
  info += QString().sprintf("Number of physical devices: %d\n",
                            window_->availablePhysicalDevices().count());

  QVulkanFunctions* f = inst->functions();
  VkPhysicalDeviceProperties props;
  f->vkGetPhysicalDeviceProperties(window_->physicalDevice(), &props);
  info += QString().sprintf(
    "Active physical device name: '%s' version %d.%d.%d\nAPI version "
    "%d.%d.%d\n",
    props.deviceName, VK_VERSION_MAJOR(props.driverVersion),
    VK_VERSION_MINOR(props.driverVersion),
    VK_VERSION_PATCH(props.driverVersion), VK_VERSION_MAJOR(props.apiVersion),
    VK_VERSION_MINOR(props.apiVersion), VK_VERSION_PATCH(props.apiVersion));

  info += QStringLiteral("Supported instance layers:\n");
  for(const QVulkanLayer& layer : inst->supportedLayers())
    info +=
      QString().sprintf("    %s v%u\n", layer.name.constData(), layer.version);
  info += QStringLiteral("Enabled instance layers:\n");
  for(const QByteArray& layer : inst->layers())
    info += QString().sprintf("    %s\n", layer.constData());

  info += QStringLiteral("Supported instance extensions:\n");
  for(const QVulkanExtension& ext : inst->supportedExtensions())
    info +=
      QString().sprintf("    %s v%u\n", ext.name.constData(), ext.version);
  info += QStringLiteral("Enabled instance extensions:\n");
  for(const QByteArray& ext : inst->extensions())
    info += QString().sprintf("    %s\n", ext.constData());

  info +=
    QString().sprintf("Color format: %u\nDepth-stencil format: %u\n",
                      window_->colorFormat(), window_->depthStencilFormat());

  info += QStringLiteral("Supported sample counts:");
  const QVector<int> sampleCounts = window_->supportedSampleCounts();
  for(int count : sampleCounts)
    info += QLatin1Char(' ') + QString::number(count);
  info += QLatin1Char('\n');

  emit static_cast<VulkanWindow*>(window_)->vulkanInfoReceived(info);
}

void VulkanRenderer::startNextFrame() {
  TriangleRenderer::startNextFrame();
  emit static_cast<VulkanWindow*>(window_)->frameQueued(int(rotation_) % 360);
}
