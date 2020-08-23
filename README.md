# roslaunchコマンド 

## 静的解析

以下のオプションを使うと，実際にノードを立ち上げずにlaunchファイルの静的解析を行う．  
基本的にroslaunchの通常実行よりはエラーになりにくく，条件は大体以下のよう．

- エラーになる場合
  - launchファイルやパラメータファイルの構文エラー
  - `$(find hoge)` や `$(env HOGE)` 等の展開エラー
  - `<include>` や `<param command="load">` 等で指定したファイルが存在しない
- エラーにならない場合
  - `<node>` に 存在しないパッケージやノード型を指定（一部のオプション（`--args`）を除く）


環境依存でビルド出来ないパッケージ・ノードがあっても，簡単なチェックには使えそう．

### `roslaunch --args <node-name>`

ノードの起動コマンドを出力する．あくまでノード単体の起動コマンドであって，パラメータ等は無視されていることに注意．  


```terminal
$ roslaunch roslaunch_sample pubsub.launch --args /sample/talker
ROS_NAMESPACE="/sample" /home/yuma/catkin_ws/devel/lib/roslaunch_sample/talker __name:=talker
```

`roscore` を立ち上げた上で以下のようにできる．

```terminal
$ roslaunch roslaunch_sample pubsub.launch --args /sample/talker | bash
[ INFO] [1598176110.135874176]: publish: Hello world!
[ INFO] [1598176110.236018332]: publish: Hello world!
[ INFO] [1598176110.336017100]: publish: Hello world!
:
:
```

このコマンドは指定したパッケージまたはノードが存在しないときエラーになる．

```
$ roslaunch roslaunch_sample not_exist_package.launch --args /dummy
Traceback (most recent call last):
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/__init__.py", line 257, in main
    node_args.print_node_args(options.node_args, args)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 108, in print_node_args
    args = get_node_args(node_name, roslaunch_files)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 194, in get_node_args
    args = create_local_process_args(node, machine)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 267, in create_local_process_args
    raise NodeParamsException(str(e))
NodeParamsException: dummy
ROS path [0]=/opt/ros/melodic/share/ros
:
:

$ roslaunch roslaunch_sample not_exist_node_type.launch --args /dummy
Traceback (most recent call last):
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/__init__.py", line 257, in main
    node_args.print_node_args(options.node_args, args)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 108, in print_node_args
    args = get_node_args(node_name, roslaunch_files)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 194, in get_node_args
    args = create_local_process_args(node, machine)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/node_args.py", line 269, in create_local_process_args
    raise NodeParamsException("Cannot locate node of type [%s] in package [%s]. Make sure file exists in package path and permission is set to executable (chmod +x)"%(node.type, node.package))
NodeParamsException: Cannot locate node of type [dummy] in package [roslaunch_sample]. Make sure file exists in package path and permission is set to executable (chmod +x)
```

### `roslaunch --dump-params`

パラメータの一覧を出力．

```terminal
roslaunch roslaunch_sample pubsub.launch --dump-params
{/sample/listner/dummy: CCC, /sample/talker/dummy: BBB, /sample/talker/message: AAA}
```

### `roslaunch --files`

自身を含めた読み込まれるlaunchファイルを列挙する．ただし複数回includeされていても区別はつかない．

```terminal
$ roslaunch roslaunch_sample empty.launch --files
/home/yuma/catkin_ws/src/roslaunch_sample/launch/empty.launch

$ roslaunch roslaunch_sample include.launch --files
/home/yuma/catkin_ws/src/roslaunch_sample/launch/include.launch
/home/yuma/catkin_ws/src/roslaunch_sample/launch/pubsub.launch
/home/yuma/catkin_ws/src/roslaunch_sample/launch/empty.launch
```

### `roslaunch --find <node-name>`

指定したノードが記述されているlaunchファイルを列挙


```terminal
$ roslaunch roslaunch_sample include.launch --find /sample/listner
/home/yuma/catkin_ws/src/roslaunch_sample/launch/pubsub.launch

$ roslaunch roslaunch_sample include.launch --find listner
ERROR: cannot find node named [/listner]. Run 
        roslaunch --nodes <files>
to see list of node names.
```

### `roslaunch --nodes`

起動されるノードの名前を列挙

```terminal
$ roslaunch roslaunch_sample pubsub.launch --nodes
/sample/listner
/sample/talker
```

### `roslaunch --ros-args`

`<arg>` で指定された引数を列挙

```terminal
$ roslaunch roslaunch_sample pubsub.launch --ros-args
Required Arguments:
  dummy2: dummy argument without default value
Optional Arguments:
  dummy (default "dummy"): dummy argument

$ roslaunch roslaunch_sample empty.launch --ros-args
No arguments.
```

## 通常起動時のオプション

### 一括指定系

全てのノードの設定を強制的に変更するオプション

|オプション|上書き内容
|-|-
|`--screen`  |`output="screen"`
|`--log`     |`output="log"`
|`--required`|`required=true`

### `--core`

masterノードのみ立ち上げる．

### `--disable-title`

デフォルトだとターミナルのタイトルが `<launchファイルのパス> <ROS_MASTER_URI>` になるが，それを抑制する．

### `--master-logger-level LOG_LEVEL`

`rosmaster.master` のログレベルを変更

### `--no-summary`

以下のようなサマリー表示を抑制

```
SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.7
 * /sample/listner/dummy: CCC
 * /sample/talker/dummy: BBB
 * /sample/talker/message: AAA

NODES
  /sample/
    listner (roslaunch_sample/listner)
    talker (roslaunch_sample/talker)
```

### `--port PORT`

ポート番号の変更

### `--skip-log-check`

logディレクトリのサイズチェックを省略するらしい．

### `--verbose/-v`

よくあるやつ

```diff
+ ... loading XML file [/opt/ros/melodic/etc/ros/roscore.xml]
+ ... executing command param [rosversion roslaunch]
+ Added parameter [/rosversion]
+ ... executing command param [rosversion -d]
+ Added parameter [/rosdistro]
+ Added core node of type [rosout/rosout] in namespace [/]
+ ... loading XML file [/home/yuma/catkin_ws/src/roslaunch_sample/launch/pubsub.launch]
+ Added parameter [/sample/talker/message]
+ Added parameter [/sample/talker/dummy]
+ Added node of type [roslaunch_sample/talker] in namespace [/sample/]
+ Added parameter [/sample/listner/dummy]
+ Added node of type [roslaunch_sample/listner] in namespace [/sample/]
started roslaunch server http://localhost:46099/
```

### `--wait`

masterノードの起動を待ってから個々のノードが立ち上げるようにする．


### 要追記

- `--child`
- `--local`
- `--server_uri`
- `--run_id`
- `--pid`
- `--numworkers`
- `--timeout`
- `--sigint-timeout`
- `--sigterm-timeout`
 
## `roslaunch-check`

https://github.com/ros/ros_comm/tree/melodic-devel/tools/roslaunch/scripts には存在するがよくわからん

## `roslaunch-complete`

補完機能用っぽい．fishだとうまく動かなくてよくわからなかった．

```
$ roslaunch-complete roslaunch_sample pubsub.launch
dummy dummy2
```

## `roslaunch-deps`

依存パッケージを調べる．  
このコマンドでは `<パッケージ名> <ファイル名>` の形で指定できないことに留意．  


```terminal
$  roslaunch-deps roslaunch_sample/launch/deps.launch
roslaunch_sample pcl_ros
```

また，`-v`で詳細な情報が得られる．

```
$ roslaunch-deps roslaunch_sample/launch/deps.launch -v
processing launch roslaunch_sample/launch/deps.launch
processing included launch /home/yuma/catkin_ws/src/roslaunch_sample/launch/pubsub.launch
pcl_ros [pcl_ros/pointcloud_to_pcd]
roslaunch_sample [roslaunch_sample/launch/deps.launch]
roslaunch_sample [roslaunch_sample/talker]
roslaunch_sample [roslaunch_sample/listner]
roslaunch_sample [/home/yuma/catkin_ws/src/roslaunch_sample/launch/pubsub.launch]
--------------------------------------------------------------------------------
roslaunch_sample pcl_ros
```

さらに，`-w`で依存関係の記述漏れが指摘される．

```terminal
$ roslaunch-deps roslaunch_sample/launch/deps.launch -w
Dependencies:
roslaunch_sample pcl_ros

Missing declarations:
roslaunch_sample/manifest.xml:
  <depend package="pcl_ros" />
```

## `roslaunch-logs`

`roscore` が立ち上がった状態で実行すると，ログの出力場所が得られる．

```terminal
$ roslaunch-logs
/home/yuma/.ros/log/de3cdbd4-e535-11ea-aa97-0c9d92c95344
