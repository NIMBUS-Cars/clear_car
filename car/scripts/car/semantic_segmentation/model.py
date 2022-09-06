import torch
import torchvision
import torch.nn.functional as F
from torch import nn

numberOfImageClassifications = 4

def double_conv(in_channels, out_channels):
    return nn.Sequential(
        nn.Conv2d(in_channels, out_channels, 3, padding=1),
        nn.ReLU(inplace=True),
        nn.Conv2d(out_channels, out_channels, 3, padding=1),
        nn.ReLU(inplace=True)
    )

class SemanticSegmentationModel(nn.Module):
    def __init__(self):
        super(SemanticSegmentationModel, self).__init__()
        self.average_pool = nn.AvgPool2d(2)
        self.up_sample = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.encoder_conv1 = double_conv(3,16)
        self.encoder_conv2 = double_conv(16,32)
        self.encoder_conv3 = double_conv(32,64)
        self.encoder_conv4 = double_conv(64,128)
        self.decoder_conv1 = double_conv(64 + 128, 64)
        self.decoder_conv2 = double_conv(32 + 64, 32)
        self.decoder_conv3 = double_conv(16 + 32, 16)
        self.end_image_conv = nn.Conv2d(16, numberOfImageClassifications, 1)

    def forward(self, x):
        conv1 = self.encoder_conv1(x)
        x = self.average_pool(conv1)
        conv2 = self.encoder_conv2(x)
        x = self.average_pool(conv2)
        conv3 = self.encoder_conv3(x)
        x = self.average_pool(conv3)
        x = self.encoder_conv4(x)
        x = self.up_sample(x)
        x = torch.cat([x, conv3], dim=1)
        x = self.decoder_conv1(x)
        x = self.up_sample(x)
        x = torch.cat([x, conv2], dim=1)
        x = self.decoder_conv2(x)
        x = self.up_sample(x)
        x = torch.cat([x,conv1],dim=1)
        x = self.decoder_conv3(x)
        x = self.end_image_conv(x)
        return x

