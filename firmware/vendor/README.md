# 純正ライブラリの置き場所

ここには、ビュートバランサー2 に付属する純正のライブラリとサンプルを置きます。
Vstone などの著作物なので、このリポジトリには含めていません。

ビルドに要るファイル:

```
firmware/vendor/
  control.c, control.h          純正サンプルの制御 (Control() など)
  cr_startup_lpc13.c            スタートアップコード
  lib/Balancer2.h
  lib/LPC13xx.h
  lib/core_cm3.h
  lib/system_LPC13xx.h
  lib/libBalancer2_lib.a        純正ライブラリ (センサー、電流制御、USB など)
  stock_main.c                  純正サンプルの main (make stock で比較用にビルドするときだけ)
```

`main.c` の `control()` は、純正の `Control()` と同じ式・同じゲインを土台にしています。
足した行には `[v5]` と書いてあります。
