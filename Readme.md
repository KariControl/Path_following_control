このqiitaの記事で書いたものをROS2 humbleで実装してみました：
※ ヨーレート制御の部分だけ少しだけシンプルにしています。
https://qiita.com/KariControl/items/78862ae6b45fcfe91126

デモ版のソフトでは

 source run_path_following.sh

を実行すると，下記のように一定曲率の目標パスに対して軌道追従する制御シミュレーンが実行されます。デモ版ソフトのため、目標パス計算もpath following controlのノード内で実行しています。
![image](https://github.com/user-attachments/assets/990da0b5-a167-4a28-b64e-37fd6dce813a)

