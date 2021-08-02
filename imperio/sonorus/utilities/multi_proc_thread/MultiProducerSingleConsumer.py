from .SingleProcessor import SingleProcessor
from .MultiProcessor import MultiProcessor
import multiprocessing as mp


class MultiProducerSingleConsumer(object):
    def __init__(
        self,
        producer_target_func,
        consumer_target_func,
        in_queue=None,
        out_queue=None,
        store_out=False,
        callback=None,
        processing_size=None,
        num_proc=max(mp.cpu_count(), 2),  # minimum 2 processes
    ):
        r"""Input --> Producer --> Consumer --> Output"""

        self.producer = MultiProcessor(
            producer_target_func,
            in_queue=in_queue,
            store_out=True,
            processing_size=processing_size,
            num_proc=num_proc,
        )

        self.consumer = SingleProcessor(
            consumer_target_func,
            in_queue=self.producer.out_queue,
            out_queue=out_queue,
            store_out=store_out,
            callback=callback,
        )

        self.in_queue = self.producer.in_queue
        self.out_queue = self.consumer.out_queue
        self.process = self.producer.process
