/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include <selfdrive/ui/sunnypilot/qt/network/sunnylink/sunnylink_client.h>

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/sunnylink/sponsor_widget.h"
#include "selfdrive/ui/sunnypilot/qt/offroad/settings/settings.h"
#include "selfdrive/ui/sunnypilot/qt/widgets/scrollview.h"

class SunnylinkPanel : public QFrame {
  Q_OBJECT

public:
  explicit SunnylinkPanel(QWidget *parent = nullptr);
  void showEvent(QShowEvent *event) override;
  void paramsRefresh(const QString&param_name, const QString&param_value);

public slots:
  void updatePanel();

private:
  Params params;
  QStackedLayout *main_layout = nullptr;
  QWidget *sunnylinkScreen = nullptr;
  ScrollViewSP *sunnylinkScroller = nullptr;
  SunnylinkSponsorPopup *status_popup;
  SunnylinkSponsorPopup *pair_popup;
  ButtonControlSP* sponsorBtn;
  ButtonControlSP* pairSponsorBtn;
  SunnylinkClient* sunnylink_client;

  ParamControl *sunnylinkEnabledBtn;
  bool is_onroad = false;
  bool is_backup = false;
  bool is_restore = false;
  bool is_sunnylink_enabled = false;
  ParamWatcher *param_watcher;
  QString sunnylinkBtnDescription;
  void stopSunnylink() const;
  void startSunnylink() const;
};
