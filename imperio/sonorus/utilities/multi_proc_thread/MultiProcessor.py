from .Processor import Processor
import multiprocessing as mp


class MultiProcessor(Processor):
    def __init__(
        self,
        target_func,
        in_queue=None,
        out_queue=None,
        store_out=False,
        callback=None,
        processing_size=None,
        num_proc=max(mp.cpu_count(), 2),  # minimum 2 processes
    ):

        super(MultiProcessor, self).__init__(
            target_func,
            in_queue=in_queue,
            out_queue=out_queue,
            store_out=store_out,
            callback=callback,
        )

        self.processing_size = processing_size if processing_size else num_proc
        self.pool = mp.Pool(processes=num_proc)

    def process(self, elem, now=False):

        if elem is not None:
            self.in_queue.put(elem)

        queue_size = self.in_queue.qsize()
        if now or queue_size == self.processing_size:

            result = self.pool.starmap(
                self.target_func, (self.in_queue.get() for i in range(queue_size))
            )

            self.callback(result)
