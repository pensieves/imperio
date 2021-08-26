from .Processor import Processor
import multiprocessing as mp


class SingleProcessor(Processor):
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

        super(SingleProcessor, self).__init__(
            target_func,
            starter_func=starter_func,
            starter_func_kwargs=starter_func_kwargs,
            in_queue=in_queue,
            out_queue=out_queue,
            store_out=store_out,
            callback=callback,
        )

        self.job = mp.Process(target=self.run)
        self.job.start()

    def process(self):
        if not self.in_queue.empty():
            result = self.target_func(self.in_queue.get())
            self.callback([result])

    def run(self):
        if self.starter_func:
            self.starter_func(**self.starter_func_kwargs)

        while True:
            self.process()
