// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill
// Lorensen SPDX-FileCopyrightText: Copyright 2004 Sandia Corporation
// SPDX-License-Identifier: LicenseRef-BSD-3-Clause-Sandia-USGov

/*========================================================================
 For general information about using VTK and Qt, see:
 http://www.trolltech.com/products/3rdparty/vtksupport.html
=========================================================================*/

// .NAME QVTKInteractorAdapter - Handle Qt events.
// .SECTION Description
// QVTKInteractor handles relaying Qt events to VTK.

#ifndef Q_VTK_INTERACTOR_ADAPTER_H
#define Q_VTK_INTERACTOR_ADAPTER_H

#include <QtCore/QObject>

class QEvent;

class vtkRenderWindowInteractor;

// .NAME EzQVTKInteractorAdapter - A QEvent translator.
// .SECTION Description
// EzQVTKInteractorAdapter translates QEvents and send them to a
// vtkRenderWindowInteractor.
class EzQVTKInteractorAdapter : public QObject {
  Q_OBJECT
public:
  // Description:
  // Constructor: takes QObject parent
  EzQVTKInteractorAdapter(QObject *parent = nullptr);

  // Description:
  // Destructor
  ~EzQVTKInteractorAdapter() override;

  // Description:
  // Set the device pixel ratio, this defaults to 1.0, but in Qt 5 can be
  // != 1.0.
  void SetDevicePixelRatio(float ratio,
                           vtkRenderWindowInteractor *iren = nullptr);
  float GetDevicePixelRatio() { return this->DevicePixelRatio; }

  // Description:
  // Process a QEvent and send it to the interactor
  // returns whether the event was recognized and processed
  bool ProcessEvent(QEvent *e, vtkRenderWindowInteractor *iren);

protected:
  int AccumulatedDelta;
  float DevicePixelRatio;
  static const double DevicePixelRatioTolerance;
};

#endif
// VTK-HeaderTest-Exclude: QVTKInteractorAdapter.h
