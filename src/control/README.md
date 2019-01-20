左のジョイスティックで移動  
L2R2(コントローラの5,6番)ボタンで旋回  
右のジョイスティックの上下でアームの上げ下げ  
Aボタン(コントローラの3番)でゲルゲを持つシリンダを伸ばす  
Bボタン(コントローラの2番)でゲルゲを持つシリンダを縮める  
Xボタン(コントローラの4番)でサーボモータを回す  
* 以下を満たすようにサーボモータが動く  
   * Xボタンを押した回数  
   * 1回目:ゲルゲを離す  
   * 2回目:シャガイ回収  
   * 3回目:シャガイ射出  
   * それ以降は2,3回目を繰り返す.  

***
メモ
***  
  Joystick.linear_x = joy.axes[1];//左のスティック上下  
  Joystick.linear_y = joy.axes[0];//左のスティック左右  

  Joystick.right_spin = joy.buttons[4];//L2ボタン(コントローラの5番)  
  Joystick.left_spin  = joy.buttons[5];//R2ボタン(コントローラの6番)  

  Joystick.arm_up = joy.axes[3];//右のスティック上下  

  Joystick.gerege_push = joy.buttons[2];//右の3番  
  Joystick.gerege_pull = joy.buttons[1];//右の2番  

  Joystick.air_servo = joy.buttons[3];//右の4番  
