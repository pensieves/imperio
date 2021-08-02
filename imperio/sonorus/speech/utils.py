import torch


def to_device(model, gpu_idx, for_eval=True, return_device=False):

    device = torch.device("cpu")
    if model is not None:

        if gpu_idx is not None and torch.cuda.is_available():
            device = torch.device(f"cuda:{gpu_idx}")

        model = model.to(device)
        if for_eval:
            model = model.eval()

    if return_device:
        return model, device
    return model
