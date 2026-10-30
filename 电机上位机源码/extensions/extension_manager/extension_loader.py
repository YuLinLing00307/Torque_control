# -*- coding:utf-8 -*-
"""
@Time: 2023/2/16 10:41
@Author: 谭国榕
@Email: guorong.tan@MotionG.com
@File: extension_loader.py
"""
# from features.io.exceptions import pyminer_exc_mgr

import os
import sys
import importlib
import json
from collections import namedtuple
import copy
import traceback
import logging
from typing import AnyStr

import utils

logger = logging.getLogger('extensionmanager.extensionloader')
# from features.extensions.extensions_manager import log


Info = namedtuple('Info',
                  ['icon',
                   'name',
                   'display_name',
                   'version',
                   'description',
                   'path',
                   'type_',
                   'group'])


class PublicInterface:
    pass


class ExtensionLoader:

    name: AnyStr
    display_name: AnyStr
    version: AnyStr
    description: AnyStr
    icon: AnyStr
    type_: AnyStr
    group: AnyStr
    path: AnyStr

    def __init__(self, manager):
        self.manager = manager
        # 这里不能直接写self.path,因为所有入口文件都叫main.py,会被Python缓存起来
        self.import_path = os.path.join(utils.get_root_dir(), 'packages')
        # 将模块导入路径设置为扩展文件夹,这样可以直接导入入口文件
        sys.path.append(self.import_path)
        self.extension_lib_path = os.path.join(utils.get_root_dir(), 'MotionGStudio', 'extensions', 'extension_lib')
        sys.path.append(self.extension_lib_path)

    def load(self, file, ui_inserters):
        self.package = json.load(file)
        self.ui_inserters = ui_inserters
        try:
            self.name = self.package['name']
            self.display_name = self.package['display_name']
            self.version = self.package['version']
            self.description = self.package['description']
            self.icon = self.package['icon']
            self.type_, self.group = self.package.get('group', '未知类型/未知分组').split('/')

            self.path = os.path.join(utils.get_root_dir(), 'packages', self.name)  # 扩展文件夹路径

            main_config = self.package.get('main', {'file': 'main.py', 'main': 'Extension'})
            main_class = self.load_class(main_config['file'], main_config['main'])
            self.main = main_class()
            from extensions.extension_lib.extension_lib import extension_lib
            self.main.extension_lib = extension_lib
            self.binding_info()
            try:
                self.main._on_loading()
            except Exception as e:
                logger.error(e, exc_info=True)

            # 获取接口配置
            interface_config = self.package.get('interface', {'file': 'main.py', 'interface': 'Interface'})
            self.main.interface = self.load_class(interface_config['file'], interface_config['interface'])()
            self.main.public_interface = self.create_public_interface(self.main.interface)

            # 扫描接口
            for key in getattr(self.main.interface, 'ui_inserters', []):
                self.ui_inserters[f'extension_{self.name}_{key}'] = self.main.interface.ui_inserters[key]

            # 加载界面相关类
            self.main.widget_classes = {}
            self.main.widgets = {}  # store auto inserted widgets
            for widget in self.package['widgets']:
                widget_class = self.load_widget(widget)
                widget_class_name = widget_class.__name__
                self.main.widget_classes[widget_class_name] = widget_class

            # 加载配置
            if 'settings' in self.package:
                assert isinstance(self.package['settings'], str)
                settings_path = os.path.join(
                    self.path, self.package['settings'])
                try:
                    with open(settings_path, 'r') as f:
                        settings = json.loads(f.read())
                except Exception as e:
                    logger.exception(e, exc_info=True)
                else:
                    self.main.settings = settings

            # todo:自动安装依赖
            # self.manager.vermanager.set_current_modules([f'{self.name}=={self.version}'])
            #
            # if 'requirements' in self.package:
            #     requirements = self.package['requirements']
            #     assert isinstance(requirements, list)
            #     solvable, conflict = self.manager.vermanager.check_requirements(
            #         requirements)
            #     if conflict:
            #         raise Exception(f'Conflicts in extensions {conflict}')
            #     if solvable:
            #         for requirement in solvable:
            #             self.manager.enable_extension(requirement)

            try:
                self.main._on_load()
            except ImportError as e:
                import traceback
                traceback.print_exc()
                if hasattr(e, 'name'):
                    command = '{executable} -m pip install {package_name}'.format(
                        package_name=e.name, executable=sys.executable)
                else:
                    command = ''
                # pyminer_exc_mgr.emit_exception_occured_signal(e, 'try install module', command)

            except Exception as e:
                logger.error(e, exc_info=True)
            return self.main
        except KeyError as e:
            logger.error(f'插件的Package.json不完整 {e}', exc_info=True)

    def binding_info(self):
        self.main.info = Info(
            name=self.name,
            icon=self.icon,
            display_name=self.display_name,
            description=self.description,
            version=self.version,
            path=self.path,
            group=self.group,
            type_=self.type_
        )

    def import_module(self, path):
        """
        import模块包
        @param path: 模块路径
        @return:
        """
        filepath = os.path.join(self.path, path)

        # pyminer_paths = [path for path in sys.path if 'pyminer' in path and path not in
        #                  (self.import_path, self.extension_lib_path)]
        # for path in pyminer_paths:
        #     sys.path.remove(path)
        # pyminer_paths = []

        try:
            package_name = self.name
            module_name = os.path.splitext(os.path.basename(filepath))[0]
            module_path = f'{package_name}.{module_name}'
            module = None
            module = importlib.import_module(module_path)

        except Exception as e:
            logger.error(e, exc_info=True)
            module = None
        # sys.path.extend(pyminer_paths)
        return module

    def load_class(self, file, class_name):
        """
        加载插件类
        @param file: 插件文件
        @param class_name: 类名
        @return:
        """
        path = os.path.join(self.path, file)
        module = self.import_module(path)
        if module:
            if hasattr(module, class_name):
                return getattr(module, class_name)
            else:
                logger.error(f"{file}文件中不存在{class_name}类", exc_info=True)
        else:
            logger.error(f"{file}文件不存在或有误", exc_info=True)

    def load_widget(self, widget_config):
        """
        加载控件
        @param widget_config: 控件配置
        @return:
        """
        try:
            widget_class = self.load_class(widget_config['file'], widget_config['widget'])
            if widget_config.get('auto_insert', True):
                widget_config = self.ui_inserters[widget_config['position']](widget_class, widget_config['config'])
                self.main.widgets[widget_class.__name__] = widget_config
            return widget_class
        except KeyError as e:
            logger.error(f"插件{self.name}的widgets配置不正确!")
            logger.error(f"位置:{widget_config}", exc_info=True)

    def create_public_interface(self, interface):
        public_interface = PublicInterface()
        for attr in dir(interface):
            obj = getattr(interface, attr)
            if not attr.startswith('_') and callable(obj):
                setattr(public_interface, attr, obj)
        return public_interface

    def reset(self):
        pass


