from pathlib import Path


def create_conf(root_dir, conf_name, **kwargs):

    with open(Path(__file__).parent / f"conf/{conf_name}.conf", "r") as f:
        conf = f.read()

    conf = conf.format(
        **{k: str(v).lower() if isinstance(v, bool) else v for k, v in kwargs.items()}
    )

    conf_dir = Path(root_dir) / "conf"
    conf_dir.mkdir(parents=True, exist_ok=True)
    conf_filename = str(conf_dir / f"{conf_name}.conf")

    with open(conf_filename, "w") as f:
        f.write(conf)
    return conf_filename


def create_mfcc_conf(
    root_dir,
    use_energy=False,
    num_mel_bins=40,
    num_ceps=40,
    low_freq=20,
    high_freq=-400,
    allow_downsample=True,
):

    return create_conf(
        root_dir,
        conf_name="mfcc",
        use_energy=use_energy,
        num_mel_bins=num_mel_bins,
        num_ceps=num_ceps,
        low_freq=low_freq,
        high_freq=high_freq,
        allow_downsample=allow_downsample,
    )


def create_ivector_extractor_conf(
    root_dir,
    cmvn_conf_kwargs=dict(),
    ivector_period=10,
    splice_conf_kwargs=dict(left_context=3, right_context=3),
    lda_matrix_root_rel_path="exp/nnet3_cleaned/extractor/final.mat",
    global_cmvn_root_rel_path="exp/nnet3_cleaned/extractor/global_cmvn.stats",
    diag_ubm_root_rel_path="exp/nnet3_cleaned/extractor/final.dubm",
    ivector_extractor_root_rel_path="exp/nnet3_cleaned/extractor/final.ie",
    num_gselect=5,
    min_post=0.025,
    posterior_scale=0.1,
    max_remembered_frames=1000,
    max_count=0,
):

    root_dir = Path(root_dir)
    cmvn_conf_path = create_online_cmvn_conf(root_dir, **cmvn_conf_kwargs)
    splice_conf_path = create_splice_conf(root_dir, **splice_conf_kwargs)
    lda_matrix_path = str(root_dir / lda_matrix_root_rel_path)
    global_cmvn_path = str(root_dir / global_cmvn_root_rel_path)
    diag_ubm_path = str(root_dir / diag_ubm_root_rel_path)
    ivector_extractor_path = str(root_dir / ivector_extractor_root_rel_path)

    return create_conf(
        root_dir,
        conf_name="ivector_extractor",
        cmvn_conf_path=cmvn_conf_path,
        ivector_period=ivector_period,
        splice_conf_path=splice_conf_path,
        lda_matrix_path=lda_matrix_path,
        global_cmvn_path=global_cmvn_path,
        diag_ubm_path=diag_ubm_path,
        ivector_extractor_path=ivector_extractor_path,
        num_gselect=num_gselect,
        min_post=min_post,
        posterior_scale=posterior_scale,
        max_remembered_frames=max_remembered_frames,
        max_count=max_count,
    )


def create_splice_conf(root_dir, left_context=3, right_context=3):
    return create_conf(
        root_dir,
        conf_name="splice",
        left_context=left_context,
        right_context=right_context,
    )


def create_online_cmvn_conf(root_dir, **kwargs):
    return create_conf(root_dir, conf_name="online_cmvn")
