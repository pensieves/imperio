from abc import ABCMeta, abstractmethod
import multiprocessing as mp


class Processor(object, metaclass=ABCMeta):
    def __init__(
        self,
        target_func,
        starter_func=None,
        starter_func_kwargs=dict(),
        in_queue=None,
        out_queue=None,
        store_out=False,
        callback=None,
    ):

        self.in_queue = in_queue if in_queue else mp.Queue()

        self.out_queue = None
        if store_out or out_queue:
            self.out_queue = out_queue if out_queue else mp.Queue()

        self.target_func = target_func

        self.starter_func = starter_func
        self.starter_func_kwargs = starter_func_kwargs

        self.callback = callback if callback else self.default_callback

    @abstractmethod
    def process(self):
        pass

    def default_callback(self, outputs):
        if self.out_queue:
            for out in outputs:
                self.out_queue.put(out)
